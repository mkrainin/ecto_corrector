/*
 * TODServiceCaller.cpp
 *
 *  Created on: Aug 9, 2011
 *      Author: mkrainin
 */

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
