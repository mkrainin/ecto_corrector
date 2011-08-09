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

namespace ecto_corrector
{
  using ecto::tendrils;

  template <typename PointT>
  struct Corrector
  {
  public:
    static void declare_params(tendrils& params)
    {
      //TODO
    }

    static void declare_io(const tendrils& /*params*/, tendrils& in, tendrils& out)
    {
      //inputs
      in.declare<geometry_msgs::PoseStamped>(
          "input_pose", "Initial object pose estimate");
      in.declare<boost::shared_ptr<const pose_corrector::RigidObjectModel> >(
          "model", "Model of object");
      in.declare<sensor_msgs::CameraInfoConstPtr>(
          "camera_info", "Camera info for rgb camera (including any ROI info)");
      in.declare<cv::Mat>(
          "depth_edges","Image representing depth discontinuities (sized to ROI)");
      in.declare<typename pcl::PointCloud<PointT>::ConstPtr >(
          "cloud","Point Cloud for current sensor frame");

      //outputs
      out.declare<geometry_msgs::PoseStamped>(
          "output_pose", "Refined object pose estimate");
    }

    void configure(tendrils& params, tendrils& /*in*/, tendrils& /*out*/)
    {
      //TODO
    }

    int process(const tendrils& in, tendrils& out)
    {
      //TODO
      return ecto::OK;
    }

  private:

  };
} //namespace

ECTO_CELL(ecto_corrector, ecto_corrector::Corrector<pcl::PointXYZ>, "CorrectorXYZ", "Performs pose refinement for rigid objects."
    " Does pose correction only; cloud conversion, depth edges, ROI computation, etc. are left to other cells");
ECTO_CELL(ecto_corrector, ecto_corrector::Corrector<pcl::PointXYZRGB>, "CorrectorXYZRGB", "Performs pose refinement for rigid objects."
    " Does pose correction only; cloud conversion, depth edges, ROI computation, etc. are left to other cells");
