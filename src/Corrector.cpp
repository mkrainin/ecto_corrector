/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "ecto/ecto.hpp"

#include "pose_corrector/corrector.h"
#include "pose_corrector/model/rigidObjectModel.h"

#include "pcl/point_cloud.h"
#include "pcl/io/io.h"

namespace ecto_corrector
{
  using ecto::tendrils;

  struct Corrector
  {
  public:
    static void declare_params(tendrils& params)
    {
      //high level params
      params.declare<int>("restarts","Number of random restarts to use",0);
      params.declare<int>("iterations","Outer loop iterations",10);
      params.declare<int>("inner_loops","Inner loop iterations",10);
      params.declare<double>("restart_max_angle","Max angle (degrees) for random restarts",30.0);
      params.declare<double>("restart_max_translation","Max translation for random restarts",0.05);

      //icp
      params.declare<bool>("use_icp","Whether to use ICP constraints",false);

      //sensor model
      params.declare<bool>("use_sensor_model","Whether to use sensor model constraints",true);
      params.declare<double>("sigma_z","Starting standard deviation for z difference",0.005);
      params.declare<double>("sigma_pixel","Starting standard deviation for pixel difference",1.0);
      params.declare<int>("window_half","Half window size for windowed sensor model (0 for non-windowed)",0);

    }

    static void declare_io(const tendrils& /*params*/, tendrils& in, tendrils& out)
    {
      //inputs
      in.declare<geometry_msgs::PoseStamped>(
          "input_pose", "Initial object pose estimate");
      in.declare<boost::shared_ptr<pose_corrector::RigidObjectModel> >(
          "model", "Model of object");
      in.declare<sensor_msgs::CameraInfoConstPtr>(
          "camera_info", "Camera info for rgb camera (including any ROI info)");
      in.declare<cv::Mat>(
          "depth_edges","Image representing depth discontinuities (sized to ROI)");
      in.declare<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr >(
          "input","Cloud to align to");
      in.declare<std::vector<std::vector<cv::Point2i> > > ("valid_segments",
             "Segments as vectors of pixel indices. Only needed if using restarts",
             std::vector<std::vector<cv::Point2i> >());
      in.declare<std::vector<cv::Point2i> >("invalid", "Pixel indices for "\
             "pixels marked as invalid by segmentation. Only needed if using restarts",
             std::vector<cv::Point2i>());

      //outputs
      out.declare<geometry_msgs::PoseStamped>(
          "output_pose", "Refined object pose estimate");
    }

    void configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      //inputs
      in_pose_ = in["input_pose"];
      model_ = in["model"];
      cam_info_ = in["camera_info"];
      depth_edges_ = in["depth_edges"];
      cloud_ = in["input"];
      valid_segments_ = in["valid_segments"];
      invalid_ = in["invalid"];

      //outputs
      out_pose_ = out["output_pose"];

      //params
      restarts_ = params["restarts"];
      iterations_ = params["iterations"];
      g2o_inner_loops_ = params["inner_loops"];
      restart_max_degrees_ = params["restart_max_angle"];
      restart_max_trans_ = params["restart_max_translation"];
      use_icp_ = params["use_icp"];
      use_sensor_model_ = params["use_sensor_model"];
      sigma_z_ = params["sigma_z"];
      sigma_pixel_ = params["sigma_pixel"];
      window_half_ = params["window_half"];

    }

    int process(const tendrils& in, const tendrils& out)
    {
      //get initial pose
      tf::Transform init_pose;
      tf::poseMsgToTF(in_pose_->pose,init_pose);

      //copy the model so there are no issues with others trying to set pose
      boost::shared_ptr<pose_corrector::RigidObjectModel> model(
          new pose_corrector::RigidObjectModel(**model_));

      //set up and run corrector
      pose_corrector::Corrector corrector;
      model->setBasePose(init_pose);
      corrector.setModel(model);
      corrector.initCamera(*cam_info_,640); //second arg should be width of pre-cropped cloud

      pose_corrector::CorrectorParams params;
      params.restarts = *restarts_;
      params.restart_max_degrees = *restart_max_degrees_;
      params.restart_max_translation = *restart_max_trans_;
      params.restart_function = pose_corrector::SEGMENTATION_SENSOR_MODEL;
      params.restart_sigma_z = *sigma_z_;
      params.iterations = *iterations_;
      params.optimizer_params.g2o_iterations = *g2o_inner_loops_;
      params.optimizer_params.use_icp_constraints = *use_icp_;
      params.optimizer_params.use_sensor_model_constraints = *use_sensor_model_;
      params.optimizer_params.sensor_model_z_st_dev = *sigma_z_;
      params.optimizer_params.sensor_model_pixel_st_dev = *sigma_pixel_;
      params.optimizer_params.sensor_model_window_half = *window_half_;
      corrector.setParams(params);

      //perform the correction
      corrector.correct(**cloud_,*depth_edges_,*valid_segments_,*invalid_);

      //set output pose
      out_pose_->header = in_pose_->header;
      tf::poseTFToMsg(model->getBasePose(),out_pose_->pose);

      return ecto::OK;
    }

  private:
    //inputs
    ecto::spore<geometry_msgs::PoseStamped> in_pose_;
    ecto::spore<boost::shared_ptr<pose_corrector::RigidObjectModel> > model_;
    ecto::spore<sensor_msgs::CameraInfoConstPtr> cam_info_;
    ecto::spore<cv::Mat> depth_edges_;
    ecto::spore<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> cloud_;
    ecto::spore<std::vector<std::vector<cv::Point2i> > > valid_segments_;
    ecto::spore<std::vector<cv::Point2i> > invalid_;

    //outputs
    ecto::spore<geometry_msgs::PoseStamped> out_pose_;

    //params
    ecto::spore<int> restarts_, iterations_, g2o_inner_loops_, window_half_;
    ecto::spore<bool> use_icp_, use_sensor_model_;
    ecto::spore<double> sigma_z_, sigma_pixel_, restart_max_degrees_, restart_max_trans_;

  };
} //namespace

ECTO_CELL(ecto_corrector, ecto_corrector::Corrector, "Corrector", "Performs pose refinement for rigid objects."
    " Does pose correction only; cloud conversion, depth edges, ROI computation, etc. are left to other cells");
