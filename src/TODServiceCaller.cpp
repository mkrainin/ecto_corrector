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

#include <ros/ros.h>

#include <tabletop_object_detector/TabletopDetection.h>


namespace ecto_corrector
{
  using ecto::tendrils;

  struct TODServiceCaller
  {
  public:
    static void declare_params(tendrils& params)
    {
      params.declare<std::string>("ply_dir","Directory containing ply files");
    }

    static void declare_io(const tendrils& /*params*/, tendrils& in, tendrils& out)
    {
      out.declare<std::string>("ply_file");
      out.declare<geometry_msgs::PoseStamped>("pose");
      out.declare<bool>("success");
    }

    void configure(const tendrils& params, const tendrils& in, const tendrils& out)
    {
      //ros stuff
      tod_service_ = nh_.serviceClient<tabletop_object_detector::TabletopDetection>("/tod");

      //params
      ply_dir_ = params.get<std::string>("ply_dir");

      //ouputs
      ply_file_ = out["ply_file"];
      pose_ = out["pose"];
      success_ = out["success"];
    }

    int process(const tendrils& in, const tendrils& out)
    {
      //call tod
      tabletop_object_detector::TabletopDetection srv;
      srv.request.num_models = 1;
      if(!tod_service_.call(srv)){
        std::cout<<"Calling tod resulted in error"<<std::endl;
        *success_ = false;
        return ecto::OK;
      }
      if(srv.response.detection.result != 4)
      {
        std::cout<<"Tod response "<<srv.response.detection.result<<" not success"<<std::endl;
        *success_ = false;
        return ecto::OK;
      }
      std::cout<<"TOD call success"<<std::endl;

      household_objects_database_msgs::DatabaseModelPose const& detection =
          srv.response.detection.models[0].model_list[0];

      //get the ply filename
      int database_num = detection.model_id;
      int object_num = database_num-18848; //copied from recognition node
      *ply_file_ = ply_dir_+str(boost::format("/object_meshed%02i.ply")%(object_num));

      //get the object pose
      *pose_ = detection.pose;

      *success_ = true;
      return ecto::OK;
    }

  private:
    //ros stuff
    ros::NodeHandle nh_;
    ros::ServiceClient tod_service_;

    //outputs
    ecto::spore<std::string> ply_file_;
    ecto::spore<geometry_msgs::PoseStamped> pose_;
    ecto::spore<bool> success_;

    //params
    std::string ply_dir_;

  };
} //namespace

ECTO_CELL(ecto_corrector, ecto_corrector::TODServiceCaller, "TODServiceCaller", "Calls TOD, returns first result as ply+pose");
