/*
 * Corrector.cpp
 *
 *  Created on: Aug 8, 2011
 *      Author: mkrainin
 */

#include "ecto/ecto.hpp"

#include "pose_corrector/corrector.h"
#include "pose_corrector/model/rigidObjectModel.h"

#include "pcl/point_cloud.h"
#include "pcl/io/io.h"

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

namespace ecto_corrector
{
  using ecto::tendrils;

  struct Corrector
  {
  public:
    static void declare_params(tendrils& params)
    {
      //high level params
      params.declare<int>("iterations","Outer loop iterations",20);
      params.declare<int>("inner_loops","Inner loop iterations",20);

      //icp
      params.declare<bool>("use_icp","Whether to use ICP constraints",false);

      //sensor model
      params.declare<bool>("use_sensor_model","Whether to use sensor model constraints",true);
      params.declare<bool>("recompute_uncertainties","Whether to recompute uncertainties for sensor model constraints",true);
      params.declare<double>("sigma_z","Starting standard deviation for z difference",0.01);
      params.declare<double>("sigma_pixel","Starting standard deviation for pixel difference",2.0);
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

      //outputs
      out_pose_ = out["output_pose"];

      //params
      iterations_ = params["iterations"];
      g2o_inner_loops_ = params["inner_loops"];
      use_icp_ = params["use_icp"];
      use_sensor_model_ = params["use_sensor_model"];
      recompute_uncertainties_ = params["recompute_uncertainties"];
      sigma_z_ = params["sigma_z"];
      sigma_pixel_ = params["sigma_pixel"];
      window_half_ = params["window_half"];

    }

    template <typename PointT>
    int process(const tendrils& in, const tendrils& out,
                boost::shared_ptr<const ::pcl::PointCloud<PointT> >& input)
    {
      //get initial pose
      tf::Transform init_pose;
      tf::poseMsgToTF(in_pose_->pose,init_pose);

      //copy the model so there are no issues with others trying to set pose
      boost::shared_ptr<pose_corrector::RigidObjectModel> model(new pose_corrector::RigidObjectModel());
      *model = **model_;

      //set up and run corrector
      pose_corrector::Corrector corrector;
      model->setBasePose(init_pose);
      corrector.setModel(model);
      corrector.initCamera(*cam_info_,input->width);

      pose_corrector::CorrectorParams params;
      params.iterations = *iterations_;
      params.optimizer_params.g2o_iterations = *g2o_inner_loops_;
      params.optimizer_params.use_icp_constraints = *use_icp_;
      params.optimizer_params.use_sensor_model_constraints = *use_sensor_model_;
      params.optimizer_params.sensor_model_recompute_uncertainties = *recompute_uncertainties_;
      params.optimizer_params.sensor_model_z_st_dev = *sigma_z_;
      params.optimizer_params.sensor_model_pixel_st_dev = *sigma_pixel_;
      params.optimizer_params.sensor_model_window_half = *window_half_;
      corrector.setParams(params);

      //put the cloud in the right format
      pcl::PointCloud<PointT> const& cloud_in = *input;
      pcl::PointCloud<pcl::PointXYZ> cloud;
      // Allocate enough space and copy the basics
      cloud.points.resize (cloud_in.points.size ());
      cloud.header   = cloud_in.header;
      cloud.width    = cloud_in.width;
      cloud.height   = cloud_in.height;
      cloud.is_dense = cloud_in.is_dense;
      for (size_t i = 0; i < cloud_in.points.size (); ++i){
        PointT const& pt = cloud_in.points[i];
        cloud.points[i].x = pt.x;
        cloud.points[i].y = pt.y;
        cloud.points[i].z = pt.z;
      }

      //perform the correction
      corrector.correct(cloud,*depth_edges_,
                        std::vector<std::vector<cv::Point2i> >(), //TODO: take in segmentation
                        std::vector<cv::Point2i>());

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

    //outputs
    ecto::spore<geometry_msgs::PoseStamped> out_pose_;

    //params
    ecto::spore<int> iterations_, g2o_inner_loops_, window_half_;
    ecto::spore<bool> use_icp_, use_sensor_model_, recompute_uncertainties_;
    ecto::spore<double> sigma_z_, sigma_pixel_;

  };
} //namespace

ECTO_CELL(ecto_corrector, pcl::PclCell<ecto_corrector::Corrector>, "Corrector", "Performs pose refinement for rigid objects."
    " Does pose correction only; cloud conversion, depth edges, ROI computation, etc. are left to other cells");
