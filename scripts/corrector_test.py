#!/usr/bin/env python
# test of componentized corrector along with todservicecaller

import ecto, ecto_ros, ecto_pcl, ecto_pcl_ros, ecto_sensor_msgs, ecto_opencv.highgui, ecto_X
import ecto_corrector
from blackboxes import *

import sys
import math

visualize = True
debug_graphs = True
dir = "/wg/stor2a/mkrainin/object_data/perception challenge/object_meshes"
rotation = 10*math.pi/180
translation = 0.03

if __name__ == "__main__":
    node_name = "corrector_test"
    ecto_ros.init(sys.argv, node_name,False)
    
    """
    
    Pose refinement subplasm
    
    """
    sub_plasm = ecto.Plasm()
    
    #subscribers/converters
    sub_cloud = ecto_sensor_msgs.Subscriber_PointCloud2("Cloud2 Subscriber",
                                        topic_name="/camera/rgb/points")
    sub_info = ecto_sensor_msgs.Subscriber_CameraInfo("Cam Info Subscriber",
                                        topic_name="/camera/rgb/camera_info")
    msg2cloud = ecto_pcl_ros.Message2PointCloud("Cloud2 To Type-Erased",
                                        format=ecto_pcl.XYZRGB)
    cloud2typed = ecto_pcl.PointCloud2PointCloudT("Type-Erased To XYZRGB",
                                                  format=ecto_pcl.XYZRGB)
    
    #artificial noise
    noise_adder = ecto_corrector.AddNoise("Noise Adder",
                                    rotation=rotation,translation=translation)

    #edge detection
    edge_detector = ecto_corrector.DepthEdgeDetector("Edge Detector",
                            depth_threshold=0.02, erode_size=3,open_size=3)
    
    #model loading
    model_loader = ecto_corrector.ModelLoader("Model Loader")
    
    #region of interest
    roi = ecto_corrector.ModelROI("ROI",expansion=50,binning=2)
    apply_roi = ecto_corrector.ApplyROI("Apply ROI")
    
    #pose correction
#    beam_corrector = ecto_corrector.Corrector("Beam Corrector",window_half=2,sigma_pixel=5.0)
    icp_corrector = ecto_corrector.Corrector("ICP Corrector",use_icp=True,use_sensor_model=False)
    
    sub_graph = [
        #conversion    
        sub_cloud[:]                >> msg2cloud[:],
        apply_roi[:]                >> cloud2typed[:],
        
        #artificial noise
        model_loader[:]             >> noise_adder["model"],
        
        #region of interest
        noise_adder["out_pose"]     >> roi["pose"],
        model_loader[:]             >> roi["model"],
        sub_info[:]                 >> roi["in_camera_info"],
        roi[:]                      >> apply_roi["info"],
        msg2cloud[:]                >> apply_roi["input"],
    
        #edge detection
        apply_roi[:]                >> edge_detector[:],                      
             
        #beam correction
#        noise_adder["out_pose"]     >> beam_corrector["input_pose"],
#        model_loader["model"]       >> beam_corrector["model"],
#        roi["out_camera_info"]      >> beam_corrector["camera_info"],
#        edge_detector["depth_edges"]>> beam_corrector["depth_edges"],
#        cloud2typed[:]              >> beam_corrector["input"],            
        
        #icp correction
        noise_adder["out_pose"]     >> icp_corrector["input_pose"],
        model_loader["model"]       >> icp_corrector["model"],
        roi["out_camera_info"]      >> icp_corrector["camera_info"],
        edge_detector["depth_edges"]>> icp_corrector["depth_edges"],
        cloud2typed[:]              >> icp_corrector["input"],      
    ]
    
    if visualize:
        pre_correct_vertices = VerticesPubModule(sub_plasm,node_name+"/pre_correct")
#        post_beam_vertices = VerticesPubModule(sub_plasm,node_name+"/post_beam")
        post_icp_vertices = VerticesPubModule(sub_plasm,node_name+"/post_icp")
        
        sub_graph += [
            #pre-correct visualization
            noise_adder["out_pose"]     >> pre_correct_vertices["pose"],
            model_loader["model"]       >> pre_correct_vertices["model"],
                      
            #beam-correct visualization
#           beam_corrector["output_pose"]>> post_beam_vertices["pose"],
#           model_loader["model"]        >> post_beam_vertices["model"],    
                      
            #icp-correct visualization
            icp_corrector["output_pose"]>> post_icp_vertices["pose"],
            model_loader["model"]       >> post_icp_vertices["model"],            
        ]

    sub_plasm.connect(sub_graph)
    if(debug_graphs):
        ecto.view_plasm(sub_plasm)
    
    #conditional executer for correction subplams
    executer = ecto_X.Executer(plasm=sub_plasm, niter=1, outputs={}, 
                        inputs={"in_pose":noise_adder,"ply_file":model_loader})
    correction_subgraph_if = ecto.If('Correction if success',cell=executer)

    """
    
    Main Plasm
    
    """
    main_plasm = ecto.Plasm()
    
    #triggering
    sub_image = ecto_sensor_msgs.Subscriber_Image("Image Subscriber",
                                        topic_name="/camera/rgb/image_color")
    img2mat = ecto_ros.Image2Mat("Image to Mat", swap_rgb=True)
    show_triggers = {'d_key':ord('d')}
    show = ecto_opencv.highgui.imshow("imshow",waitKey=10,triggers=show_triggers)
    trigger_and = ecto.And("Trigger And",ninput=2)
    
    #object detection
    tod_detector = ecto_corrector.TODServiceCaller("TOD",ply_dir=dir)
    tod_detector_if = ecto.If('TOD if key',cell=tod_detector)
    
    main_graph = [                  
        #display
        sub_image[:]    >>  img2mat[:],    
        img2mat[:]      >>  show["input"],
        
        #triggering
        show["d_key"]               >>  (tod_detector_if["__test__"],trigger_and["in1"]),
        tod_detector_if["success"]  >>  trigger_and["in2"],
        trigger_and["out"]          >>  correction_subgraph_if["__test__"],
        
        #correction subgraph
        tod_detector_if["ply_file","pose"]>>correction_subgraph_if["ply_file","in_pose"],            
     ]
    main_plasm.connect(main_graph)
    if(debug_graphs):
        ecto.view_plasm(main_plasm)
    
    
    #run the plasm
    print "Using subscribers + TOD for inputs"
    print "Ensure OpenNI node and TOD node are running"
    print "Press 'd' in imshow window to detect. 'q' to quit"
    sched = ecto.schedulers.Singlethreaded(main_plasm)
    sched.execute()

