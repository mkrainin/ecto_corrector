/*
 * CameraInfoSender.cpp
 *
 *  Created on: Jul 26, 2011
 *      Author: mkrainin
 */

#include "ecto/ecto.hpp"
#include "sensor_msgs/CameraInfo.h"

namespace ecto_corrector
{
  using ecto::tendrils;

  struct CameraInfoSender
  {
  public:
    static void declare_params(tendrils& params)
    {
      params.declare<std::string>("frame_id", "tf frame associated with camera");
      params.declare<int>("width", "image width");
      params.declare<int>("height", "image height");
      params.declare<double>("center_x", "center x");
      params.declare<double>("center_y", "center y");
      params.declare<double>("focal_x", "x focal length");
      params.declare<double>("focal_y", "y focal length");
    }

    static void declare_io(const tendrils& /*params*/, tendrils& in, tendrils& out)
    {
      out.declare<sensor_msgs::CameraInfoPtr>("camera_info", "Camera info");
    }

    void configure(tendrils& params, tendrils& /*in*/, tendrils& /*out*/)
    {
      frame=params["frame_id"];
      width=params["width"];
      height=params["height"];
      cx=params["center_x"];
      cy=params["center_y"];
      fx=params["focal_x"];
      fy=params["focal_y"];
    }

    int process(const tendrils& in, tendrils& out)
    {
      sensor_msgs::CameraInfoPtr info(new sensor_msgs::CameraInfo());

      info->header.frame_id = *frame;
      info->header.stamp = ros::Time::now();
      info->width = *width;
      info->height = *height;
      info->K[2] = info->P[2] = *cx;
      info->K[5] = info->P[6] = *cy;
      info->K[0] = info->P[0] = *fx;
      info->K[4] = info->P[5] = *fy;
      info->K[8] = info->P[10] = 1;

      out.get<sensor_msgs::CameraInfoPtr>("camera_info") = info;

      return ecto::OK;
    }

  private:
    ecto::spore<std::string> frame;
    ecto::spore<double> cx,cy,fx,fy;
    ecto::spore<int> width,height;
  };
} //namespace
ECTO_CELL(ecto_corrector, ecto_corrector::CameraInfoSender, "CameraInfoSender", "Outputs a manually set sensor_msgs::CameraInfo");
