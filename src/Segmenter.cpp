/*
 * Segmenter.cpp
 *
 *  Created on: Aug 26, 2011
 *      Author: mkrainin
 */

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
