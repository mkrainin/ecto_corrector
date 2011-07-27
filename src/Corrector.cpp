/*
 * Corrector.cpp
 *
 *  Created on: Jul 19, 2011
 *      Author: mkrainin
 */

#include "ecto/ecto.hpp"

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

#include "pose_corrector/corrector.h"
#include "pose_corrector/model/rigidObjectModel.h"

namespace ecto_corrector
{
  using ecto::tendrils;

  struct Corrector
  {
  public:
    static void declare_params(tendrils& params)
    {
      //TODO: iterations, refine, etc
      //e.g. params.declare<std::string>("prefix", "A prefix for printing");
    }

    static void declare_io(const tendrils& /*params*/, tendrils& in, tendrils& out)
    {
      in.declare<geometry_msgs::PoseStamped>(
          "input_pose", "Initial object pose estimate", geometry_msgs::PoseStamped());
      in.declare<std::string>(
          "ply_file", "PLY file w/ triangulated mesh", "");
      in.declare<sensor_msgs::CameraInfoConstPtr>(
          "camera_info", "Camera info for rgb camera", sensor_msgs::CameraInfoConstPtr());
      in.declare<sensor_msgs::ImageConstPtr>(
          "image_color","Image for current sensor frame",sensor_msgs::ImageConstPtr());
      in.declare<sensor_msgs::PointCloud2ConstPtr>(
          "cloud","Point Cloud for current sensor frame",sensor_msgs::PointCloud2ConstPtr());
      out.declare<geometry_msgs::PoseStamped>(
          "output_pose", "Refined object pose estimate");
    }

    void configure(tendrils& params, tendrils& /*in*/, tendrils& /*out*/)
    {
      //TODO: copy out params/update structures
    }

    int process(const tendrils& in, tendrils& out)
    {
      tf::Transform init_pose;
      tf::poseMsgToTF(in.get<geometry_msgs::PoseStamped>("input_pose").pose,init_pose);

      //TODO: some of these structures should persist
      pose_corrector::Corrector corrector;
      boost::shared_ptr<pose_corrector::BaseModel> model(
          new pose_corrector::RigidObjectModel(in.get<std::string>("ply_file")));
      corrector.setModel(model);
      corrector.setModelBasePose(init_pose);
      corrector.initCamera(in.get<sensor_msgs::CameraInfoConstPtr>("camera_info"));

      corrector.correct(in.get<sensor_msgs::ImageConstPtr>("image_color"),
                        in.get<sensor_msgs::PointCloud2ConstPtr>("cloud"));

      geometry_msgs::PoseStamped final_pose;
      final_pose.header = in.get<geometry_msgs::PoseStamped>("input_pose").header;
      tf::poseTFToMsg(corrector.getModelBasePose(),final_pose.pose);

      out.get<geometry_msgs::PoseStamped>("output_pose") = final_pose;

      return ecto::OK;
    }

  private:
    pose_corrector::OptimizerG2OParams opt_params;
  };
} //namespace
ECTO_CELL(ecto_corrector, ecto_corrector::Corrector, "Corrector", "Performs pose refinement for rigid objects");
