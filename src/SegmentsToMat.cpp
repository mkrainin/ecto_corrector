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

#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>


namespace ecto_corrector
{
  using ecto::tendrils;

  struct SegmentsToMat
  {
  public:
    static void declare_params(tendrils& params)
    {
      params.declare<int>("min_size","Minimum segment size to visualize",100);
    }

    static void declare_io(const tendrils& /*params*/, tendrils& in, tendrils& out)
    {
      //inputs
      in.declare<std::vector<std::vector<cv::Point2i> > >("segments", "Segments to visualize");

      //outputs
      out.declare<cv::Mat>("output", "Image with different colors for each segment");
    }

    void configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      //inputs
      valid_segments_ = in["segments"];

      //outputs
      mat_ = out["output"];

      //params
      min_size_ = params["min_size"];
    }

    template <typename PointT>
    int process(const tendrils& in, const tendrils& out,
                boost::shared_ptr<const ::pcl::PointCloud<PointT> >& input)
    {
      *mat_ = pose_corrector::visualizeSegments(input->width,
                                                input->height,
                                                *valid_segments_);
      return ecto::OK;
    }

  private:
    //inputs
    ecto::spore<std::vector<std::vector<cv::Point2i> > > valid_segments_;

    //outputs
    ecto::spore<cv::Mat> mat_;

    //params
    ecto::spore<int> min_size_;

  };
} //namespace

ECTO_CELL(ecto_corrector, pcl::PclCell<ecto_corrector::SegmentsToMat>,
          "SegmentsToMat","Provides a visual representation for image segment");
