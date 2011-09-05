/*
 * ModelVertices.cpp
 *
 *  Created on: Aug 16, 2011
 *      Author: mkrainin
 */

#include <ecto/ecto.hpp>

#include <pcl/registration/transforms.h>

#include <pose_corrector/model/rigidObjectModel.h>


namespace ecto_corrector
{
  using ecto::tendrils;

  struct ModelVertices
  {
  public:
    static void declare_params(tendrils& params)
    {
    }

    static void declare_io(const tendrils& /*params*/, tendrils& in, tendrils& out)
    {
      in.declare<boost::shared_ptr<pose_corrector::RigidObjectModel> >("model", "Model whose vertices are to be extracted");
      in.declare<geometry_msgs::PoseStamped>("pose", "Pose of model");
      out.declare<sensor_msgs::PointCloud2ConstPtr>("cloud", "PointCloud2 with vertices");
    }

    void configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      model_ = in["model"];
      pose_ = in["pose"];
      cloud_ = out["cloud"];
    }

    int process(const tendrils& in, const tendrils& out)
    {
      //get cloud
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = (*model_)->getVertexCloud();

      //transform
      geometry_msgs::Point t = pose_->pose.position;
      geometry_msgs::Quaternion q = pose_->pose.orientation;
      pcl::transformPointCloud(*cloud,*cloud,Eigen::Vector3f(t.x,t.y,t.z),Eigen::Quaternionf(q.w,q.x,q.y,q.z));

      cloud->header = pose_->header;

      sensor_msgs::PointCloud2Ptr cloud2(new sensor_msgs::PointCloud2());
      pcl::toROSMsg(*cloud,*cloud2);
      *cloud_ = cloud2;

      return ecto::OK;
    }

  private:
    //inputs
    ecto::spore<boost::shared_ptr<pose_corrector::RigidObjectModel> > model_;
    ecto::spore<geometry_msgs::PoseStamped> pose_;

    //outputs
    ecto::spore<sensor_msgs::PointCloud2ConstPtr > cloud_;

  };
} //namespace

ECTO_CELL(ecto_corrector, ecto_corrector::ModelVertices, "ModelVertices", "Gets a point cloud with the vertices of a model transformed appropriately");
