#!/usr/bin/env python

import ecto, ecto_ros, ecto_pcl, ecto_pcl_ros, ecto_sensor_msgs, ecto_opencv.highgui
import ecto_corrector

import sys

debug = False

Cloud2Sub = ecto_sensor_msgs.Subscriber_PointCloud2
InfoSub = ecto_sensor_msgs.Subscriber_CameraInfo
Converter = ecto_pcl_ros.Message2PointCloud

if __name__ == "__main__":
    ecto_ros.init(sys.argv, "depth_edge_test")
    
    sub_cloud = Cloud2Sub("cloud sub",topic_name="/camera/rgb/points")
    converter = Converter("cloud converter",format=ecto_pcl.XYZ)
    sub_info = InfoSub("info sub",topic_name="/camera/rgb/camera_info")
    edge_detector = ecto_corrector.DepthEdgeDetector("Edge Detector",depth_threshold=0.02, \
          erode_size=3,open_size=3)
    im_drawer = ecto_opencv.highgui.imshow("Drawer",name="depth edges", waitKey=10)
    
    graph = [
                    sub_cloud[:] >> converter[:],
                    converter[:] >> edge_detector["cloud"],
                    sub_info[:]  >> edge_detector["cam_info"],
                    edge_detector[:] >> im_drawer[:]
             ]

    plasm = ecto.Plasm()
    plasm.connect(graph)
    
    if(debug):
        ecto.view_plasm(plasm)
    
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()   
