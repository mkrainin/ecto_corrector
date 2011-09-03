/*
 * ApplyROI.cpp
 *
 *  Created on: Sep 2, 2011
 *      Author: mkrainin
 */

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
