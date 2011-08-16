#!/usr/bin/env python
# test of componentized corrector along with todservicecaller

import ecto, ecto_ros, ecto_pcl, ecto_pcl_ros, ecto_sensor_msgs, ecto_opencv.highgui
import ecto_corrector
from blackboxes import *

import sys
import math

debug = False

if __name__ == "__main__":
    node_name = "corrector_test"
    ecto_ros.init(sys.argv, node_name,False)
    
    plasm = ecto.Plasm()
    
    #inputs
    inputs = InputsModule(plasm,rot_noise=15*math.pi/180,trans_noise=0.05)
    
    #pose correction
    beam_corrector = ecto_corrector.CorrectorXYZ("Beam Corrector",window_half=2,sigma_pixel=5.0)
    icp_corrector = ecto_corrector.CorrectorXYZ("ICP Corrector",use_icp=True,use_sensor_model=False)
    
    #display
    pre_correct_vertices = VerticesPubModule(plasm,node_name+"/pre_correct")
    post_beam_vertices = VerticesPubModule(plasm,node_name+"/post_beam")
    post_icp_vertices = VerticesPubModule(plasm,node_name+"/post_icp")
    
    graph = [
        #pre-correct visualization
        inputs["pose"]          >> pre_correct_vertices["pose"],
        inputs["model"]         >> pre_correct_vertices["model"],
             
        #beam correction
        inputs["pose"]          >> beam_corrector["input_pose"],
        inputs["model"]         >> beam_corrector["model"],
        inputs["info"]          >> beam_corrector["camera_info"],
        inputs["depth_edges"]   >> beam_corrector["depth_edges"],
        inputs["cloud"]         >> beam_corrector["cloud"],         
        
        #beam-correct visualization
        beam_corrector["output_pose"]>> post_beam_vertices["pose"],
        inputs["model"]              >> post_beam_vertices["model"],
        
        #icp correction
        inputs["pose"]          >> icp_corrector["input_pose"],
        inputs["model"]         >> icp_corrector["model"],
        inputs["info"]          >> icp_corrector["camera_info"],
        inputs["depth_edges"]   >> icp_corrector["depth_edges"],
        inputs["cloud"]         >> icp_corrector["cloud"],  
        
        #icp-correct visualization
        icp_corrector["output_pose"]>> post_icp_vertices["pose"],
        inputs["model"]             >> post_icp_vertices["model"],
    ]

    plasm.connect(graph)
    
    if(debug):
        ecto.view_plasm(plasm)
    
    sched = ecto.schedulers.Threadpool(plasm)
    
    print "Press enter to run once. 'q' to quit"
    result = 0
    while(raw_input() != 'q' and result == 0):
        print "Running Once"
        result = sched.execute(1)   
        if(result == 0):
            print "Done. Press enter to run again. 'q' to quit"
        else:
            print "Execution failed (probably no TOD detections)"
