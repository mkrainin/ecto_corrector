#!/usr/bin/env python
# test of componentized corrector along with todservicecaller

import ecto, ecto_ros, ecto_pcl, ecto_pcl_ros, ecto_sensor_msgs, ecto_opencv.highgui
import ecto_corrector
from blackboxes import *

import sys

debug = True

if __name__ == "__main__":
    node_name = "corrector_test"
    ecto_ros.init(sys.argv, node_name,False)
    
    plasm = ecto.Plasm()
    
    #inputs
    inputs = InputsModule(plasm)
    
    #pose correction
    corrector = ecto_corrector.CorrectorXYZ("Corrector")  #TODO: params
    
    #display
    pre_correct_vertices = VerticesPubModule(plasm,node_name+"/pre_correct")
    post_correct_vertices = VerticesPubModule(plasm,node_name+"/post_correct")
    
    graph = [
        #pre-correct visualization
        inputs["pose"]          >> pre_correct_vertices["pose"],
        inputs["model"]         >> pre_correct_vertices["model"],
             
        #correction
        inputs["pose"]          >> corrector["input_pose"],
        inputs["model"]         >> corrector["model"],
        inputs["info"]          >> corrector["camera_info"],
        inputs["depth_edges"]   >> corrector["depth_edges"],
        inputs["cloud"]         >> corrector["cloud"],         
        
        #pose-correct visualization
        corrector["output_pose"]>> post_correct_vertices["pose"],
        inputs["model"]         >> post_correct_vertices["model"],
    ]

    plasm.connect(graph)
    
    if(debug):
        ecto.view_plasm(plasm)
    
    sched = ecto.schedulers.Threadpool(plasm)
    
    print "Press enter to run once. 'q' to quit"
    while(raw_input() != 'q'):
        print "Running Once"
        sched.execute(1)   
        print "Done. Press enter to run again. 'q' to quit"