/*
 * ModelLoader.cpp
 *
 *  Created on: Aug 8, 2011
 *      Author: mkrainin
 */

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

    void configure(tendrils& params, tendrils& in, tendrils& out)
    {
      ply_file_ = in["ply_file"];
      out_model_ = out["model"];
    }

    int process(const tendrils& in, tendrils& out)
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
