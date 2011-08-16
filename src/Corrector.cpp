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
      in.declare<boost::shared_ptr<pose_corrector::RigidObjectModel> >(
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

    void configure(tendrils& params, tendrils& in, tendrils& out)
    {
      //inputs
      in_pose_ = in["input_pose"];
      model_ = in["model"];
      cam_info_ = in["camera_info"];
      depth_edges_ = in["depth_edges"];
      cloud_ = in["cloud"];

      //outputs
      out_pose_ = out["output_pose"];

      //params

    }

    int process(const tendrils& in, tendrils& out)
    {
      //get initial pose
      tf::Transform init_pose;
      tf::poseMsgToTF(in_pose_->pose,init_pose);

      //set up and run corrector
      pose_corrector::Corrector corrector;
      (*model_)->setBasePose(init_pose);
      corrector.setModel(*model_);
      corrector.initCamera(*cam_info_);

      corrector.setParams(params_);

      //put the cloud in the right format
      pcl::PointCloud<PointT> const& cloud_in = **cloud_;
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
      corrector.correct(cloud,*depth_edges_);

      //set output pose
      out_pose_->header = in_pose_->header;
      tf::poseTFToMsg((*model_)->getBasePose(),out_pose_->pose);

      return ecto::OK;
    }

  private:
    //inputs
    ecto::spore<geometry_msgs::PoseStamped> in_pose_;
    ecto::spore<boost::shared_ptr<pose_corrector::RigidObjectModel> > model_;
    ecto::spore<sensor_msgs::CameraInfoConstPtr> cam_info_;
    ecto::spore<cv::Mat> depth_edges_;
    ecto::spore<typename pcl::PointCloud<PointT>::ConstPtr > cloud_;

    //outputs
    ecto::spore<geometry_msgs::PoseStamped> out_pose_;

    //params
    pose_corrector::CorrectorParams params_;

  };
} //namespace

ECTO_CELL(ecto_corrector, ecto_corrector::Corrector<pcl::PointXYZ>, "CorrectorXYZ", "Performs pose refinement for rigid objects."
    " Does pose correction only; cloud conversion, depth edges, ROI computation, etc. are left to other cells");
ECTO_CELL(ecto_corrector, ecto_corrector::Corrector<pcl::PointXYZRGB>, "CorrectorXYZRGB", "Performs pose refinement for rigid objects."
    " Does pose correction only; cloud conversion, depth edges, ROI computation, etc. are left to other cells");
