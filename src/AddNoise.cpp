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

#include <pose_corrector/model/rigidObjectModel.h>
#include <pose_corrector/utils.h>


namespace ecto_corrector
{
  using ecto::tendrils;

  struct AddNoise
  {
  public:
    static void declare_params(tendrils& params)
    {
      params.declare<double>("rotation","Max rotation (radians) to add to pose",0.0);
      params.declare<double>("translation","Max translation (meters) to add to pose",0.0);
    }

    static void declare_io(const tendrils& /*params*/, tendrils& in, tendrils& out)
    {
      //inputs
      in.declare<boost::shared_ptr<pose_corrector::RigidObjectModel> >("model", "Model whose pose we're adding noise to");
      in.declare<geometry_msgs::PoseStamped>("in_pose", "Pose of model");

      //outputs
      out.declare<geometry_msgs::PoseStamped>("out_pose", "Pose of model with noise added");
    }

    void configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      //inputs
      model_ = in["model"];
      pose_in_ = in["in_pose"];

      //outputs
      pose_out_ = out["out_pose"];

      //params
      rot_ = params["rotation"];
      trans_ = params["translation"];
    }

    int process(const tendrils& in, const tendrils& out)
    {
      tf::Transform init_pose, init_reparam, result;
      tf::poseMsgToTF(pose_in_->pose,init_pose);
      init_reparam = (*model_)->convertToReparameterizedPose(init_pose);

      //add noise
      tf::Transform noise;
      pose_corrector::getRandomTransform(*trans_,*trans_,*trans_,*rot_,noise);
      result = (*model_)->convertFromReparameterizedPose(init_reparam*noise);

      pose_out_->header = pose_in_->header;
      tf::poseTFToMsg(result,pose_out_->pose);

      return ecto::OK;
    }

  private:
    //inputs
    ecto::spore<boost::shared_ptr<pose_corrector::RigidObjectModel> > model_;
    ecto::spore<geometry_msgs::PoseStamped> pose_in_;

    //outputs
    ecto::spore<geometry_msgs::PoseStamped> pose_out_;

    //params
    ecto::spore<double> rot_, trans_;

  };
} //namespace

ECTO_CELL(ecto_corrector, ecto_corrector::AddNoise, "AddNoise", "Add noise to the pose of a model");
