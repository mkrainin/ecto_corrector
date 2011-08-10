#!/usr/bin/env python
# test of the monolithic corrector (for now just on a fake detection)

import ecto
import ecto_ros, ecto_sensor_msgs
import ecto_corrector

import sys

ImageSub = ecto_sensor_msgs.Subscriber_Image
Cloud2Sub = ecto_sensor_msgs.Subscriber_PointCloud2
InfoSub = ecto_sensor_msgs.Subscriber_CameraInfo

if __name__ == "__main__":
    ecto_ros.init(sys.argv, "full_corrector_test")

    ply_file = "/u/mkrainin/object_data/coke_can_scaled.ply"
    fake_detection = ecto_corrector.DetectionSimulator("Fake Detection", 
            ply=ply_file, x=0, y=0.05, z=0.78, qx=0.91, qy=-0.01, qz=-0.02, qw=0.42)
    sub_rgb = ImageSub("image sub",topic_name="/camera/rgb/image_color")
    sub_cloud = Cloud2Sub("cloud sub",topic_name="/camera/rgb/points")
    sub_info = InfoSub("info sub",topic_name="/camera/rgb/camera_info")
    corrector = ecto_corrector.FullCorrector("Corrector")
    
    graph = [
                    sub_rgb[:] >> corrector["image_color"],
                    sub_cloud[:] >> corrector["cloud"],
                    sub_info[:] >> corrector["camera_info"],
                    fake_detection["ply_file","output_pose"] >> corrector["ply_file","input_pose"]
             ]

    plasm = ecto.Plasm()
    plasm.connect(graph)
    
    ecto.view_plasm(plasm)
    
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()   
