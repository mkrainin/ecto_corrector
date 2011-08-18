#!/usr/bin/env python
# writes out bags with object id and pose

import ecto, ecto_ros, ecto_pcl, ecto_pcl_ros, ecto_sensor_msgs, ecto_geometry_msgs
from ecto_opencv import highgui,calib,imgproc
import ecto_corrector
from blackboxes import *

import sys
import math

debug = False

if __name__ == "__main__":
    if(len(sys.argv) != 2 or sys.argv[1] == "-h" or sys.argv[1] == "--help"):
        print "\nUsage: "+sys.argv[0]+" out_bag\n"
        sys.exit()
    
    bag_name = sys.argv[1]
    node_name = "labeled_example_writer"
    ecto_ros.init(sys.argv, node_name,False)
    
    #subscriptions
    subs = dict(image=ecto_sensor_msgs.Subscriber_Image(topic_name='/camera/rgb/image_color', queue_size=1),
            info=ecto_sensor_msgs.Subscriber_CameraInfo(topic_name='/camera/rgb/camera_info', queue_size=1),
            cloud=ecto_sensor_msgs.Subscriber_PointCloud2(topic_name='/camera/rgb/points', queue_size=1),
            )
    sync = ecto_ros.Synchronizer('Synchronizator', subs=subs)
    sub_keys = subs.keys()
    
    #conversions
    img2mat = ecto_ros.Image2Mat("Image to Mat", swap_rgb=True)
    rgb2gray = imgproc.cvtColor('RGB -> Gray', flag=imgproc.Conversion.RGB2GRAY)
    info2cv = ecto_ros.CameraInfo2Cv("Info to CV")
    to_pose_stamped = ecto_ros.RT2PoseStamped("To Pose Stamped",frame_id="openni_rgb_optical_frame")
    
    #pose estimation
    rows = 5; cols = 3; square_size=0.04; pattern_type=calib.ASYMMETRIC_CIRCLES_GRID
    circle_detector = calib.PatternDetector(rows=rows, cols=cols,
                                        pattern_type=pattern_type,
                                        square_size=square_size)
    poser = calib.FiducialPoseFinder()
    offset = ecto_corrector.TranslationOffset("Offset",tx=-0.079,ty=0.120,tz=0.000)

    #visualization
    circle_drawer = calib.PatternDrawer(rows=rows, cols=cols)
    pose_drawer = calib.PoseDrawer()
    show = highgui.imshow("imshow",waitKey=10)
    
    #bagging
    baggers = dict(image=ecto_sensor_msgs.Bagger_Image(topic_name='/image_color'),
               info=ecto_sensor_msgs.Bagger_CameraInfo(topic_name='/camera_info'),
               cloud=ecto_sensor_msgs.Bagger_PointCloud2(topic_name="/points"),
               pose=ecto_geometry_msgs.Bagger_PoseStamped(topic_name="/pose"),
               )
    bagwriterif = ecto.If('Bag Writer if key',
                        cell=ecto_ros.BagWriter(baggers=baggers, bag=bag_name)
                        )

    plasm = ecto.Plasm()
    plasm.connect(
        #conversions
        sync["image"]  >>  img2mat[:],
        sync["info"]   >>  info2cv[:],
        
        #dot pattern
        img2mat[:]   >>  (rgb2gray[:], circle_drawer["input"]),
        rgb2gray[:]   >>  circle_detector["input"],
        circle_detector["out", "ideal", "found"] >> poser["points", "ideal", "found"],
        info2cv["K"]  >> poser["K"],
        #offset
        poser["T","R"] >> offset["T","R"],
        
        #pose publishing
        poser["R"] >> to_pose_stamped["R"], 
        offset["out"] >> to_pose_stamped["T"],
        
        #visualization
        circle_detector["out","found"]    >>  circle_drawer["points","found"],
        poser["R"] >> pose_drawer["R"], offset["out"] >> pose_drawer["T"],
        circle_drawer["out"] >> pose_drawer["image"],
        info2cv["K"] >> pose_drawer["K"],
        pose_drawer["output"] >> show["input"],
        
        #bagging
        sync[sub_keys]       >>  bagwriterif[sub_keys],
        to_pose_stamped["pose"]  >> bagwriterif["pose"],
    )
    
    if(debug):
        ecto.view_plasm(plasm)
    
    sched = ecto.schedulers.Singlethreaded(plasm)
    quit = sched.execute(1)
    
    print "press 'c' (in the imshow window) to capture a frame. 'q' to quit."
    while(not quit):
        quit = sched.execute(1)
        bagwriterif.inputs.__test__ = False
        if(show.outputs.out == ord('c')):
            print "'c' pressed. recording next frame"
            show.outputs.out = 0
            bagwriterif.inputs.__test__ = True
            