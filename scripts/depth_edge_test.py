#!/usr/bin/env python

import ecto, ecto_ros, ecto_pcl, ecto_pcl_ros, ecto_sensor_msgs, ecto_opencv.highgui
import ecto_corrector

import sys

debug = True

if __name__ == "__main__":
    ecto_ros.init(sys.argv, "depth_edge_test")
    
    #subscribers
    sub_cloud = ecto_sensor_msgs.Subscriber_PointCloud2("Cloud2 Subscriber",topic_name="/camera/rgb/points")
    sub_info = ecto_sensor_msgs.Subscriber_CameraInfo("Cam Info Subscriber",topic_name="/camera/rgb/camera_info")
    
    #converters (PointCloud2 -> type erased PointCloud -> PointCloud<PointXYZ>)
    msg2cloud = ecto_pcl_ros.Message2PointCloud("Cloud2 To Type-Erased",format=ecto_pcl.XYZ)
    cloud2typed = ecto_pcl.PointCloud2PointCloudT("Type-Erased To XYZ",format=ecto_pcl.XYZ)

    #detection
    edge_detector = ecto_corrector.DepthEdgeDetectorXYZ("Edge Detector",depth_threshold=0.02, \
          erode_size=3,open_size=3)
    
    #drawing
    im_drawer = ecto_opencv.highgui.imshow("Drawer",name="depth edges", waitKey=10)
    
    graph = [
                    sub_cloud[:] >> msg2cloud[:],
                    msg2cloud[:] >> cloud2typed[:],
                    cloud2typed[:] >> edge_detector["cloud"],
                    sub_info[:]  >> edge_detector["cam_info"],
                    edge_detector[:] >> im_drawer[:]
             ]

    plasm = ecto.Plasm()
    plasm.connect(graph)
    
    if(debug):
        ecto.view_plasm(plasm)
    
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()   
