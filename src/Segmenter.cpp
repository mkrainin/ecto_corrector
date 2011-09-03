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
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    //output
    outputs.declare<cv::Mat> ("segment_image", "Image showing the resulting segments in different colors");
  }

  void configure(const ecto::tendrils& params, const ecto::tendrils& inputs, const ecto::tendrils& outputs)
  {
    //params
    pixel_step_ = params["pixel_step"];
    depth_threshold_ = params["depth_threshold"];
    normal_threshold_ = params["normal_threshold"];
    curvature_threshold_ = params["curvature_threshold"];

    //output
    segment_image_ = outputs["segment_image"];
  }

  template <typename PointT>
  int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs,
              boost::shared_ptr<const ::pcl::PointCloud<PointT> >& input,
              boost::shared_ptr<const ::pcl::PointCloud< ::pcl::Normal> >& normals)
  {
    std::vector<std::vector<cv::Point2i> > valid_segments;
    std::vector<cv::Point2i> invalid;
    pose_corrector::segmentCloud(*input,*normals,valid_segments,invalid,
       *pixel_step_,*depth_threshold_,*normal_threshold_,*curvature_threshold_);

    *segment_image_ = pose_corrector::visualizeSegments(input->width,input->height,valid_segments);

    return ecto::OK;
  }

  //params
  ecto::spore<int> pixel_step_;
  ecto::spore<double> depth_threshold_;
  ecto::spore<double> normal_threshold_;
  ecto::spore<double> curvature_threshold_;

  //outputs
  ecto::spore<cv::Mat> segment_image_;

};

}

ECTO_CELL(ecto_corrector, pcl::PclCellWithNormals<ecto_corrector::Segmenter>,
          "Segmenter", "Segments a pcl::PointCloud based on depth similarity of neighboring pixels");
