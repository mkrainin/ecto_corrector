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

struct ApplyROI
{
  ApplyROI(){}

  static void declare_params(ecto::tendrils& params)
  {
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    //inputs
    //Note: input cloud handled by PclCell
    inputs.declare<sensor_msgs::CameraInfoConstPtr>("info", "CameraInfo containing ROI to use");

    //output
    outputs.declare<ecto::pcl::PointCloud> ("output", "Cloud with ROI applied");
  }

  void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
  {
    //inputs
    info_ = inputs["info"];

    //output
    output_ = outputs["output"];
  }

  template <typename PointT>
  int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs,
              boost::shared_ptr<const ::pcl::PointCloud<PointT> >& input)
  {
    typename pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>());

    pose_corrector::Camera cam(*info_);
    pose_corrector::applyROI(*input,cam.getReducedResROI(),*out);

    *output_ = ecto::pcl::xyz_cloud_variant_t(out);

    return ecto::OK;
  }

  //inputs
  ecto::spore<sensor_msgs::CameraInfoConstPtr> info_;

  //outputs
  ecto::spore<ecto::pcl::PointCloud> output_;

};

}

ECTO_CELL(ecto_corrector, pcl::PclCell<ecto_corrector::ApplyROI>,
          "ApplyROI", "Applies a region of interest to a PointCloud");
