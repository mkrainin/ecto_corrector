/*
 * ModelROI.cpp
 *
 *  Created on: Aug 8, 2011
 *      Author: mkrainin
 */

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
    }

    static void declare_io(const tendrils& /*params*/, tendrils& in, tendrils& out)
    {
      //inputs
      in.declare<sensor_msgs::CameraInfoConstPtr>(
          "in_camera_info", "Camera info before ROI");
      in.declare<boost::shared_ptr<const pose_corrector::RigidObjectModel> >(
          "model", "Model for the input ply");
      in.declare<geometry_msgs::PoseStamped>(
          "pose", "Estimated pose of the object");

      //outputs
      out.declare<sensor_msgs::CameraInfoPtr>(
          "out_camera_info", "Camera info with appropriate ROI");
    }

    void configure(tendrils& params, tendrils& in, tendrils& out)
    {
      //inputs
      in_info_ = in["in_camera_info"];
      model_ = in["model"];
      pose_ = in["pose"];

      //outputs
      out_info_ = out["out_camera_info"];

      //params
      expansion_ = params["expansion"];
    }

    int process(const tendrils& in, tendrils& out)
    {
      tf::Transform tf_pose;
      tf::poseMsgToTF(pose_->pose,tf_pose);
      pose_corrector::Camera cam(*in_info_,false);

      sensor_msgs::RegionOfInterest roi = (*model_)->getCameraROI(cam,tf_pose,*expansion_);

      *out_info_ = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo());
      **out_info_ = **in_info_;
      (*out_info_)->roi = roi;


      return ecto::OK;
    }

  private:
    //inputs
    ecto::spore<sensor_msgs::CameraInfoConstPtr> in_info_;
    ecto::spore<boost::shared_ptr<const pose_corrector::RigidObjectModel> > model_;
    ecto::spore<geometry_msgs::PoseStamped> pose_;

    //outputs
    ecto::spore<sensor_msgs::CameraInfoPtr> out_info_;

    //params
    ecto::spore<int> expansion_;

  };
} //namespace

ECTO_CELL(ecto_corrector, ecto_corrector::ModelROI, "ModelROI", "Sets camera Region Of Interest (ROI) based on model+pose");
