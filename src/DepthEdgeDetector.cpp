/*
 * DepthEdgeDetector.cpp
 *
 *  Created on: Aug 7, 2011
 *      Author: mkrainin
 */

#include <ecto/ecto.hpp>

#include <pose_corrector/utils.h>
#include <pcl/point_cloud.h>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

namespace ecto_corrector{

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
    //Note: input cloud handled by PclCell

    //output
    outputs.declare<cv::Mat> ("depth_edges", "Image depicting depth discontinuities");
  }

  void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
  {
    //params
    depth_threshold_ = params["depth_threshold"];
    erode_size_ = params["erode_size"];
    open_size_ = params["open_size"];

    //output
    output_ = outputs["depth_edges"];
  }

  template <typename PointT>
  int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs,
              boost::shared_ptr<const ::pcl::PointCloud<PointT> >& input)
  {
    pose_corrector::computeDepthEdges(*input,*output_,*depth_threshold_,*erode_size_,*open_size_);
    return ecto::OK;
  }

  //params
  ecto::spore<double> depth_threshold_;
  ecto::spore<int> erode_size_;
  ecto::spore<int> open_size_;

  //outputs
  ecto::spore<cv::Mat> output_;

};

}

ECTO_CELL(ecto_corrector, pcl::PclCell<ecto_corrector::DepthEdgeDetector>,
          "DepthEdgeDetector", "Detects depth discontinuities in a PointCloud and produces a cv::Mat");
