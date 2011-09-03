#!/usr/bin/env python

import ecto, ecto_ros
from ecto_pcl import NiConverter
from ecto_openni import Capture, ResolutionMode, Device
from ecto_opencv.highgui import imshow
from ecto_corrector import DepthEdgeDetector

import sys

debug = True

#openni
device = Capture('ni device', rgb_resolution=ResolutionMode.VGA_RES,
           depth_resolution=ResolutionMode.VGA_RES,
           rgb_fps=30, depth_fps=30,
           device_number=0,
           registration=True,
           synchronize=False,
           device=Device.KINECT
           )
cloud_generator = NiConverter('cloud_generator')
graph = [   device[:] >> cloud_generator[:], ]

#detection
edge_detector = DepthEdgeDetector("Edge Detector",depth_threshold=0.02, \
      erode_size=3,open_size=3)
graph += [cloud_generator[:] >> edge_detector["input"], ]

#drawing
im_drawer = imshow("Drawer",name="depth edges", waitKey=10)
graph += [edge_detector[:] >> im_drawer[:]]

plasm = ecto.Plasm()
plasm.connect(graph)

if __name__ == "__main__":
    
    if(debug):
        ecto.view_plasm(plasm)
    
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute()   
