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
