#!/usr/bin/env python

import ecto, ecto_ros
from ecto_openni import Capture, ResolutionMode, Device
from ecto_pcl import NiConverter, NormalEstimation, KDTREE_ORGANIZED_INDEX
from ecto_corrector import Segmenter, SegmentsToMat
from ecto_opencv.highgui import imshow

import sys

debug = False

#openni capture
device = Capture('ni device', rgb_resolution=ResolutionMode.VGA_RES,
               depth_resolution=ResolutionMode.VGA_RES,
               rgb_fps=30, depth_fps=30,
               device_number=0,
               registration=True,
               synchronize=False,
               device=Device.KINECT
               )
cloud_generator = NiConverter('cloud_generator')
graph =     [   device[:] >> cloud_generator[:] ]

#normals
normals = NormalEstimation("Normals", k_search=0, radius_search=0.006,
                           spatial_locator=KDTREE_ORGANIZED_INDEX)
graph +=    [   cloud_generator[:]  >> normals[:]   ]

#segmentation
segmenter = Segmenter("Segmenter",pixel_step=3,
                      depth_threshold=0.0015, #0.0015
                      normal_threshold=0.98, #0.96
                      curvature_threshold=10, #not using curvature threshold
                      max_depth = 0.9)

graph +=    [   cloud_generator[:]  >> segmenter["input"],
                normals[:]          >> segmenter["normals"],    ]

#drawing
seg2mat = SegmentsToMat("Seg2Mat",min_size=10)
im_drawer = imshow("Drawer",name="segments", waitKey=10)
graph +=    [   segmenter["valid_segments"] >> seg2mat["segments"],
                cloud_generator[:]          >> seg2mat["input"],
                seg2mat[:]                  >>  im_drawer[:],
             ]


plasm = ecto.Plasm()
plasm.connect(graph)

if __name__ == "__main__":
    if(debug):
        ecto.view_plasm(plasm)
    
    sched = ecto.schedulers.Threadpool(plasm)

    if debug:
        sched.execute_async()
        from IPython.Shell import IPShellEmbed
        ipshell = IPShellEmbed()
        ipshell()
        
    else:
        sched.execute()
