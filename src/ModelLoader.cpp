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

namespace ecto_corrector
{
  using ecto::tendrils;

  struct ModelLoader
  {
  public:
    static void declare_params(tendrils& params)
    {
    }

    static void declare_io(const tendrils& /*params*/, tendrils& in, tendrils& out)
    {
      //inputs
      in.declare<std::string>(
          "ply_file", "PLY file w/ triangulated mesh", "");

      //outputs
      out.declare<boost::shared_ptr<pose_corrector::RigidObjectModel> >(
          "model", "Model for the input ply");
    }

    void configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      ply_file_ = in["ply_file"];
      out_model_ = out["model"];
    }

    int process(const tendrils& in, const tendrils& out)
    {
      if(models_.find(*ply_file_) == models_.end()){
        boost::shared_ptr<pose_corrector::RigidObjectModel> model(
            new pose_corrector::RigidObjectModel(*ply_file_));
        models_[*ply_file_] = model;
      }

      *out_model_ = models_[*ply_file_];
      return ecto::OK;
    }

  private:
    ecto::spore<std::string> ply_file_;
    ecto::spore<boost::shared_ptr<pose_corrector::RigidObjectModel> > out_model_;

    std::map<std::string,boost::shared_ptr<pose_corrector::RigidObjectModel> > models_;

  };
} //namespace

ECTO_CELL(ecto_corrector, ecto_corrector::ModelLoader, "ModelLoader", "Manages loading of ply models");
