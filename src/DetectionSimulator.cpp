/*
 * DetectionSimulator.cpp
 *
 *  Created on: Jul 26, 2011
 *      Author: mkrainin
 */

#include "ecto/ecto.hpp"
#include "geometry_msgs/PoseStamped.h"

namespace ecto_corrector
{
  using ecto::tendrils;

  struct DetectionSimulator
  {
  public:
    static void declare_params(tendrils& params)
    {
      params.declare<std::string>("ply", "PLY file w/ triangulated mesh");
      params.declare<double>("x", "x component of the pose of the object wrt camera");
      params.declare<double>("y", "y component of the pose of the object wrt camera");
      params.declare<double>("z", "z component of the pose of the object wrt camera");
      params.declare<double>("qx", "qx component of the pose of the object wrt camera");
      params.declare<double>("qy", "qy component of the pose of the object wrt camera");
      params.declare<double>("qz", "qz component of the pose of the object wrt camera");
      params.declare<double>("qw", "qw component of the pose of the object wrt camera");
    }

    static void declare_io(const tendrils& /*params*/, tendrils& in, tendrils& out)
    {
      out.declare<std::string>("ply_file", "PLY file w/ triangulated mesh");
      out.declare<geometry_msgs::PoseStamped>("output_pose", "Pose estimate");
    }

    void configure(const tendrils& params, const tendrils& /*in*/, const tendrils& /*out*/)
    {
      ply=params["ply"];
      x=params["x"];
      y=params["y"];
      z=params["z"];
      qx=params["qx"];
      qy=params["qy"];
      qz=params["qz"];
      qw=params["qw"];
    }

    int process(const tendrils& in, const tendrils& out)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "/object";
      pose.pose.position.x = *x;
      pose.pose.position.y = *y;
      pose.pose.position.z = *z;
      pose.pose.orientation.x = *qx;
      pose.pose.orientation.y = *qy;
      pose.pose.orientation.z = *qz;
      pose.pose.orientation.w = *qw;

      out.get<std::string>("ply_file") = *ply;
      out.get<geometry_msgs::PoseStamped>("output_pose") = pose;

      return ecto::OK;
    }

  private:
    ecto::spore<std::string> ply;
    ecto::spore<double> x,y,z,qx,qy,qz,qw;
  };
} //namespace
ECTO_CELL(ecto_corrector, ecto_corrector::DetectionSimulator, "DetectionSimulator", "Outputs ply and pose that were set manually");
