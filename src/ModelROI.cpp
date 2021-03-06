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

#include <ecto/ecto.hpp>

#include <pose_corrector/model/rigidObjectModel.h>

namespace ecto_corrector
{
  using ecto::tendrils;

  struct ModelROI
  {
  public:
    static void declare_params(tendrils& params)
    {
      params.declare<int>("expansion","Number of pixels to expand ROI in each direction");
      params.declare<int>("binning","Binning to use in output camera pose",1);
    }

    static void declare_io(const tendrils& /*params*/, tendrils& in, tendrils& out)
    {
      //inputs
      in.declare<sensor_msgs::CameraInfoConstPtr>(
          "in_camera_info", "Camera info before ROI");
      in.declare<boost::shared_ptr<pose_corrector::RigidObjectModel> >(
          "model", "Model for the input ply");
      in.declare<geometry_msgs::PoseStamped>(
          "pose", "Estimated pose of the object");

      //outputs
      out.declare<sensor_msgs::CameraInfoConstPtr>(
          "out_camera_info", "Camera info with appropriate ROI");
    }

    void configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      //inputs
      in_info_ = in["in_camera_info"];
      model_ = in["model"];
      pose_ = in["pose"];

      //outputs
      out_info_ = out["out_camera_info"];

      //params
      expansion_ = params["expansion"];
      binning_ = params["binning"];
    }

    int process(const tendrils& in, const tendrils& out)
    {
      tf::Transform tf_pose;
      tf::poseMsgToTF(pose_->pose,tf_pose);
      pose_corrector::Camera cam(*in_info_,false);

      sensor_msgs::RegionOfInterest roi = (*model_)->getCameraROI(cam,tf_pose,*expansion_);

      sensor_msgs::CameraInfoPtr new_info(new sensor_msgs::CameraInfo());
      *new_info = **in_info_;
      new_info->roi = roi;
      new_info->binning_x = *binning_;
      new_info->binning_y = *binning_;

      //pass through Camera to resolve any alignment issues caused by the binning
      pose_corrector::Camera binned_camera(new_info);

      *out_info_ = binned_camera.getCameraInfo();
      return ecto::OK;
    }

  private:
    //inputs
    ecto::spore<sensor_msgs::CameraInfoConstPtr> in_info_;
    ecto::spore<boost::shared_ptr<pose_corrector::RigidObjectModel> > model_;
    ecto::spore<geometry_msgs::PoseStamped> pose_;

    //outputs
    ecto::spore<sensor_msgs::CameraInfoConstPtr> out_info_;

    //params
    ecto::spore<int> expansion_;
    ecto::spore<int> binning_;

  };
} //namespace

ECTO_CELL(ecto_corrector, ecto_corrector::ModelROI, "ModelROI", "Sets camera Region Of Interest (ROI) based on model+pose");
