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
#include <opencv2/core/core.hpp>

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell_with_normals.hpp>

namespace ecto_corrector{

struct Segmenter
{
  Segmenter(){}

  static void declare_params(ecto::tendrils& params)
  {
    params.declare<int>("pixel_step", "number of pixels over to compare to",2);
    params.declare<double>("depth_threshold", "depth difference that constitutes a discontinuity",0.005);
    params.declare<double>("normal_threshold", "max dot product between adjacent normals",0.95);
    params.declare<double>("curvature_threshold", "max curvature",0.04);
    params.declare<double>("max_depth","max depth to bother segmenting",2.0);
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    //inputs handled by PclCellWithNormals

    //output
    outputs.declare<std::vector<std::vector<cv::Point2i> > > ("valid_segments",
           "Segments as vectors of pixel indices. Every valid pixel within "\
           "max_depth will be in exactly one of these");
    outputs.declare<std::vector<cv::Point2i> >("invalid", "Pixel indices for "\
           "pixels with either invalid readings or beyond max_depth");
  }

  void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
  {
    //params
    pixel_step_ = params["pixel_step"];
    depth_threshold_ = params["depth_threshold"];
    normal_threshold_ = params["normal_threshold"];
    curvature_threshold_ = params["curvature_threshold"];
    max_depth_ = params["max_depth"];

    //output
    valid_segments_ = outputs["valid_segments"];
    invalid_ = outputs["invalid"];
  }

  template <typename PointT>
  int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs,
              boost::shared_ptr<const ::pcl::PointCloud<PointT> >& input,
              boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
  {
    pose_corrector::segmentCloud(*input,*normals,*valid_segments_,*invalid_,
       *pixel_step_,*depth_threshold_,*normal_threshold_,*curvature_threshold_,
       *max_depth_);

    return ecto::OK;
  }

  //params
  ecto::spore<int> pixel_step_;
  ecto::spore<double> depth_threshold_;
  ecto::spore<double> normal_threshold_;
  ecto::spore<double> curvature_threshold_;
  ecto::spore<double> max_depth_;

  //outputs
  ecto::spore<std::vector<std::vector<cv::Point2i> > > valid_segments_;
  ecto::spore<std::vector<cv::Point2i> > invalid_;
};

}

ECTO_CELL(ecto_corrector, pcl::PclCellWithNormals<ecto_corrector::Segmenter>,
          "Segmenter", "Segments a pcl::PointCloud based on depth similarity of neighboring pixels");
