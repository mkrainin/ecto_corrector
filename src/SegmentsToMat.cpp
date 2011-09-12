/*
 * SegmentsToMat.cpp
 *
 *  Created on: Sep 12, 2011
 *      Author: mkrainin
 */

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
