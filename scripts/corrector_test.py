#!/usr/bin/env python
# test of componentized corrector along with todservicecaller

import ecto, ecto_ros, ecto_pcl, ecto_pcl_ros, ecto_sensor_msgs, ecto_opencv.highgui
import ecto_corrector

import sys

debug = True
dir = "/wg/stor2a/mkrainin/object_data/perception challenge/object_meshes"

if __name__ == "__main__":
    ecto_ros.init(sys.argv, "depth_edge_test")
    
    #subscribers
    sub_cloud = ecto_sensor_msgs.Subscriber_PointCloud2("Cloud2 Subscriber",topic_name="/camera/rgb/points")
    sub_info = ecto_sensor_msgs.Subscriber_CameraInfo("Cam Info Subscriber",topic_name="/camera/rgb/camera_info")
    
    #converters (PointCloud2 -> type erased PointCloud -> PointCloud<PointXYZ>)
    msg2cloud = ecto_pcl_ros.Message2PointCloud("Cloud2 To Type-Erased",format=ecto_pcl.XYZ)
    cloud2typed = ecto_pcl.PointCloud2PointCloudT("Type-Erased To XYZ",format=ecto_pcl.XYZ)
    
    #object detection
    tod_detector = ecto_corrector.TODServiceCaller("TOD",ply_dir=dir)

    #edge detection
    edge_detector = ecto_corrector.DepthEdgeDetectorXYZ("Edge Detector",depth_threshold=0.02, \
          erode_size=3,open_size=3)
    
    #model loading
    model_loader = ecto_corrector.ModelLoader("Model Loader")
    
    #region of interest
    roi = ecto_corrector.ModelROI("ROI",expansion=40)
    
    #pose correction
    corrector = ecto_corrector.CorrectorXYZ("Corrector")  #TODO: params
    
    graph = [
        #conversion    
        sub_cloud[:]            >> msg2cloud[:],
        msg2cloud[:]            >> cloud2typed[:],
        
        #model loading
        tod_detector["ply_file"]>> model_loader[:],
        
        #region of interest
        tod_detector["pose"]    >> roi["pose"],
        model_loader[:]         >> roi["model"],
        sub_info[:]             >> roi["in_camera_info"],
    
        #edge detection
        cloud2typed[:]          >> edge_detector["cloud"],
        roi[:]                  >> edge_detector["cam_info"],
        
        #correction
        tod_detector["pose"]    >> corrector["input_pose"],
        model_loader[:]         >> corrector["model"],
        roi[:]                  >> corrector["camera_info"],
        edge_detector[:]        >> corrector["depth_edges"],
        cloud2typed[:]          >> corrector["cloud"],          
    ]

    plasm = ecto.Plasm()
    plasm.connect(graph)
    
    if(debug):
        ecto.view_plasm(plasm)
    
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()   