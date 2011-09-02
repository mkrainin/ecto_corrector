#!/usr/bin/env python

import ecto, ecto_ros, ecto_pcl, ecto_pcl_ros, ecto_sensor_msgs, ecto_opencv.highgui
import ecto_corrector

import sys

debug = False

if __name__ == "__main__":
    ecto_ros.init(sys.argv, "segmenter_test")
    
    #subscribers
    sub_cloud = ecto_sensor_msgs.Subscriber_PointCloud2("Cloud2 Subscriber",topic_name="/camera/rgb/points")
    
    #converters (PointCloud2 -> type erased PointCloud -> PointCloud<PointXYZ>)
    msg2cloud = ecto_pcl_ros.Message2PointCloud("Cloud2 To Type-Erased",format=ecto_pcl.XYZ)
    #cloud2typed = ecto_pcl.PointCloud2PointCloudT("Type-Erased To XYZ",format=ecto_pcl.XYZ)
    
    #normals
    normals = ecto_pcl.NormalEstimation("Normals", k_search=0, radius_search=0.006, spatial_locator=ecto_pcl.KDTREE_ORGANIZED_INDEX)
    #normals = ecto_pcl.NormalEstimation("Normals", k_search=20, radius_search=0, spatial_locator=ecto_pcl.KDTREE_FLANN)

    #segmentation
    segmenter = ecto_corrector.Segmenter("Segmenter",pixel_step=2,
                                                     depth_threshold=0.0015, #0.0015
                                                     normal_threshold=0.98, #0.96
                                                     curvature_threshold=10) #not using curvature threshold
    
    #drawing
    im_drawer = ecto_opencv.highgui.imshow("Drawer",name="segments", waitKey=10)
    
    graph = [
                    sub_cloud[:] >> msg2cloud[:],
                    msg2cloud[:] >> normals[:],
                    msg2cloud[:] >> segmenter["input"],
                    normals[:]   >> segmenter["normals"],
                    segmenter[:] >> im_drawer[:]
             ]

    plasm = ecto.Plasm()
    plasm.connect(graph)
    
    if(debug):
        ecto.view_plasm(plasm)
    
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()   
