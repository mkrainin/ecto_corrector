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
