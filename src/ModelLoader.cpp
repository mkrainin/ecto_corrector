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
      //TODO
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

    void configure(tendrils& params, tendrils& /*in*/, tendrils& /*out*/)
    {
      //TODO
    }

    int process(const tendrils& in, tendrils& out)
    {
      //TODO
      return ecto::OK;
    }

  private:

  };
} //namespace

ECTO_CELL(ecto_corrector, ecto_corrector::ModelLoader, "ModelLoader", "Manages loading of ply models");
