/*
 * DepthEdgeDetector.cpp
 *
 *  Created on: Aug 7, 2011
 *      Author: mkrainin
 */

#include <ecto/ecto.hpp>
#include "ecto_corrector.hpp"

#include <pose_corrector/utils.h>
#include <pcl/point_cloud.h>

template <typename PointT>
class FuncCaller
{
public:
  cv::Mat callFunction(pcl::PointCloud<PointT> const& cloud)
  {
    cv::Mat out;
    pose_corrector::computeDepthEdges(cloud,*camera_,out,depth_threshold_,erode_size_,open_size_);
    return out;
  }

  void setParams(sensor_msgs::CameraInfoConstPtr cam_info,
                 double depth_threshold, int erode_size, int open_size)
  {
    camera_ = boost::shared_ptr<pose_corrector::Camera>(new pose_corrector::Camera(cam_info,false));
    depth_threshold_ = depth_threshold;
    erode_size_ = erode_size;
    open_size_ = open_size;
  }

private:
  boost::shared_ptr<pose_corrector::Camera> camera_;
  double depth_threshold_;
  int erode_size_, open_size_;

};

#define DECLARE_MYFUNC(r, data, i, ELEM)                            \
  BOOST_PP_COMMA_IF(i) FuncCaller< BOOST_PP_TUPLE_ELEM(2, 0, ELEM) >

typedef boost::variant< BOOST_PP_SEQ_FOR_EACH_I(DECLARE_MYFUNC, ~, ECTO_XYZ_POINT_TYPES) > my_func_variant_t;


namespace ecto_corrector{

struct DepthEdgeDetector
{
  typedef cv::Mat out_type;

  /* used to create a surface */
  struct make_my_func_variant : boost::static_visitor<my_func_variant_t>
  {
    template <typename CloudType >
    my_func_variant_t operator()(const CloudType& p) const
    {
      return my_func_variant_t(FuncCaller<typename CloudType::element_type::PointType>());
    }
  };

  /* dispatch to handle process */
  struct func_dispatch : boost::static_visitor<out_type>
  {
    template <typename Func, typename CloudType>
    out_type operator()(Func& f, CloudType& i) const
    {
      return impl(f, i, pcl_takes_point_trait<Func, CloudType>());
    }

    template <typename Func, typename CloudType>
    out_type impl(Func& f, boost::shared_ptr<const CloudType>& i, boost::true_type) const
    {
      f.setParams(cam_info_,depth_threshold_,erode_size_,open_size_);
      return f.callFunction(*i);
    }

    template <typename Surface, typename CloudType>
    out_type impl(Surface& f, CloudType& i, boost::false_type) const
    {
      throw std::runtime_error("types aren't the same, you are doing something baaaaaad");
    }

    void setParams(sensor_msgs::CameraInfoConstPtr cam_info,
                 double depth_threshold, int erode_size, int open_size)
    {
      cam_info_ = cam_info;
      depth_threshold_ = depth_threshold;
      erode_size_ = erode_size;
      open_size_ = open_size;
    }

    sensor_msgs::CameraInfoConstPtr cam_info_;
    double depth_threshold_;
    int erode_size_;
    int open_size_;
  };

  static void declare_params(ecto::tendrils& params)
  {
    params.declare<double>("depth_threshold", "depth difference that constitutes a discontinuity",0.02);
    params.declare<int>("erode_size", "Element size for expanding edge (should be odd)",3);
    params.declare<int>("open_size", "Element size for filling holes in edges (should be odd)",3);
  }

  static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
  {
    //input
    inputs.declare<PointCloud> ("cloud", "The cloud to filter");
    inputs.declare<sensor_msgs::CameraInfoConstPtr> ("cam_info", "Camera info");

    //output
    outputs.declare<out_type> ("depth_edges", "Image depicting depth discontinuities");
  }

  DepthEdgeDetector() : configured_(false) {}

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
    xyz_cloud_variant_t cvar = cloud_->make_variant();
    if(!configured_){
      impl_ = boost::apply_visitor(make_my_func_variant(), cvar);
      configured_ = true;
    }
    func_dispatch disp;
    disp.setParams(*info_,*depth_threshold_,*erode_size_,*open_size_);
    *output_ = boost::apply_visitor(disp, impl_, cvar);
//    std::cout<<"Outputting Mat with size: "<<output_->cols<<","<<output_->rows<<std::endl;
//    std::cout<<"Empty?: "<<output_->empty()<<std::endl;
    return 0;
  }

  bool configured_;
  my_func_variant_t impl_;

  //params
  ecto::spore<double> depth_threshold_;
  ecto::spore<int> erode_size_;
  ecto::spore<int> open_size_;

  //inputs
  ecto::spore<PointCloud> cloud_;
  ecto::spore<sensor_msgs::CameraInfoConstPtr> info_;

  //outputs
  ecto::spore<out_type> output_;

};

}

ECTO_CELL(ecto_corrector, ecto_corrector::DepthEdgeDetector, "DepthEdgeDetector", "Detects depth discontinuities in a point cloud and produces a cv::Mat");
