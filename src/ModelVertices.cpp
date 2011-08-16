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
      in.declare<boost::shared_ptr<const pose_corrector::RigidObjectModel> >("model");
      in.declare<geometry_msgs::PoseStamped>("pose");
      out.declare<pcl::PointCloud<pcl::PointXYZ>::Ptr >("cloud");
    }

    void configure(tendrils& params, tendrils& in, tendrils& out)
    {
      model_ = in["model"];
      pose_ = in["pose"];
      cloud_ = out["cloud"];
    }

    int process(const tendrils& in, tendrils& out)
    {
      //get cloud
      *cloud_ = (*model_)->getVertexCloud();

      //transform
      geometry_msgs::Point t = pose_->pose.position;
      geometry_msgs::Quaternion q = pose_->pose.orientation;
      pcl::transformPointCloud(**cloud_,**cloud_,Eigen::Vector3f(t.x,t.y,t.z),Eigen::Quaternionf(q.w,q.x,q.y,q.z));

      (*cloud_)->header = pose_->header;

      return ecto::OK;
    }

  private:
    //inputs
    ecto::spore<boost::shared_ptr<const pose_corrector::RigidObjectModel> > model_;
    ecto::spore<geometry_msgs::PoseStamped> pose_;

    //outputs
    ecto::spore<pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_;

  };
} //namespace

ECTO_CELL(ecto_corrector, ecto_corrector::ModelVertices, "ModelVertices", "Gets a point cloud with the vertices of a model transformed appropriately");
