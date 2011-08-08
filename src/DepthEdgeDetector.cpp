/*
 * DepthEdgeDetector.cpp
 *
 *  Created on: Aug 7, 2011
 *      Author: mkrainin
 */

#include <ecto/ecto.hpp>

#include <pose_corrector/utils.h>
#include <pcl/point_cloud.h>

namespace ecto_corrector{

template <typename PointT>
struct DepthEdgeDetector
{
  DepthEdgeDetector(){}

  static void declare_params(ecto::tendrils& params)
  {
    params.declare<double>("depth_threshold", "depth difference that constitutes a discontinuity",0.02);
    params.declare<int>("erode_size", "Element size for expanding edge (should be odd)",3);
    params.declare<int>("open_size", "Element size for filling holes in edges (should be odd)",3);
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    //input
    inputs.declare<typename pcl::PointCloud<PointT>::ConstPtr> ("cloud", "The cloud to filter");
    inputs.declare<sensor_msgs::CameraInfoConstPtr> ("cam_info", "Camera info");

    //output
    outputs.declare<cv::Mat> ("depth_edges", "Image depicting depth discontinuities");
  }

  void configure(ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    //params
    depth_threshold_ = params["depth_threshold"];
    erode_size_ = params["erode_size"];
    open_size_ = params["open_size"];

    //input
    cloud_ = inputs["cloud"];
    info_ = inputs["cam_info"];

    //output
    output_ = outputs["depth_edges"];
  }

  int process(const ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    pose_corrector::Camera camera(*info_,false);
    pose_corrector::computeDepthEdges(**cloud_,camera,*output_,*depth_threshold_,*erode_size_,*open_size_);
//    std::cout<<"Got Mat with size "<<output_->cols<<" by "<<output_->rows<<std::endl;
    return ecto::OK;
  }

  //params
  ecto::spore<double> depth_threshold_;
  ecto::spore<int> erode_size_;
  ecto::spore<int> open_size_;

  //inputs
  ecto::spore<typename pcl::PointCloud<PointT>::ConstPtr > cloud_;
  ecto::spore<sensor_msgs::CameraInfoConstPtr> info_;

  //outputs
  ecto::spore<cv::Mat> output_;

};

}

ECTO_CELL(ecto_corrector, ecto_corrector::DepthEdgeDetector<pcl::PointXYZ>,
          "DepthEdgeDetectorXYZ", "Detects depth discontinuities in a pcl::PointCloud<pcl::PointXYZ> and produces a cv::Mat");
ECTO_CELL(ecto_corrector, ecto_corrector::DepthEdgeDetector<pcl::PointXYZRGB>,
          "DepthEdgeDetectorXYZRGB", "Detects depth discontinuities in a pcl::PointCloud<pcl::PointXYZRGB> and produces a cv::Mat");
