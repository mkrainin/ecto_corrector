#!/usr/bin/env python
import ecto, ecto_ros, ecto_pcl, ecto_pcl_ros, ecto_sensor_msgs, ecto_opencv.highgui
import ecto_test
import ecto_corrector

dir = "/wg/stor2a/mkrainin/object_data/perception challenge/object_meshes"

class InputsModule(ecto.BlackBox):
    def __init__(self,plasm,rot_noise=0,trans_noise=0):
        ecto.BlackBox.__init__(self, plasm)
        
        #subscribers
        self.sub_cloud = ecto_sensor_msgs.Subscriber_PointCloud2("Cloud2 Subscriber",topic_name="/camera/rgb/points")
        self.sub_info = ecto_sensor_msgs.Subscriber_CameraInfo("Cam Info Subscriber",topic_name="/camera/rgb/camera_info")
        
        #converters (PointCloud2 -> type erased PointCloud -> PointCloud<PointXYZ>)
        self.msg2cloud = ecto_pcl_ros.Message2PointCloud("Cloud2 To Type-Erased",format=ecto_pcl.XYZ)
        self.cloud2typed = ecto_pcl.PointCloud2PointCloudT("Type-Erased To XYZ",format=ecto_pcl.XYZ)
        
        #object detection
        self.tod_detector = ecto_corrector.TODServiceCaller("TOD",ply_dir=dir)
        
        #artificial noise
        self.noise_adder = ecto_corrector.AddNoise("Noise Adder",rotation=rot_noise,translation=trans_noise)
    
        #edge detection
        self.edge_detector = ecto_corrector.DepthEdgeDetectorXYZ("Edge Detector",depth_threshold=0.02, \
              erode_size=3,open_size=3)
        
        #model loading
        self.model_loader = ecto_corrector.ModelLoader("Model Loader")
        
        #region of interest
        self.roi = ecto_corrector.ModelROI("ROI",expansion=40)
        
        print "Using subscribers + TOD for inputs"
        print "Ensure OpenNI node and TOD node are running"
        
        
    def expose_outputs(self):
        return {
                 "pose":self.noise_adder["out_pose"],
                 "model":self.model_loader["model"],
                 "info":self.roi["out_camera_info"],
                 "depth_edges":self.edge_detector["depth_edges"],
                 "cloud":self.cloud2typed[:]
                }
        
    def expose_parameters(self):
        return {
                }
        
    def connections(self):
        return [
            #conversion    
            self.sub_cloud[:]            >> self.msg2cloud[:],
            self.msg2cloud[:]            >> self.cloud2typed[:],
            
            #model loading
            self.tod_detector["ply_file"]>> self.model_loader[:],
            
            #artificial noise
            self.tod_detector["pose"]   >> self.noise_adder["in_pose"],
            self.model_loader[:]        >> self.noise_adder["model"],
            
            #region of interest
            self.noise_adder["out_pose"] >> self.roi["pose"],
            self.model_loader[:]         >> self.roi["model"],
            self.sub_info[:]             >> self.roi["in_camera_info"],
        
            #edge detection
            self.cloud2typed[:]          >> self.edge_detector["cloud"],
            self.roi[:]                  >> self.edge_detector["cam_info"],
        ]
        
class VerticesPubModule(ecto.BlackBox):
    def __init__(self,plasm,topic):
        ecto.BlackBox.__init__(self, plasm)
        
        #vertex retriever
        self.model_vertices = ecto_corrector.ModelVertices()
        
        #publisher
        self.pub_cloud = ecto_sensor_msgs.Publisher_PointCloud2(topic_name=topic)
        
        
    def expose_inputs(self):
        return {
                 "model":self.model_vertices["model"],
                 "pose":self.model_vertices["pose"],
                }
        
    def expose_parameters(self):
        return {
                }
        
    def connections(self):
        return [
            self.model_vertices["cloud"] >> self.pub_cloud[:]
        ]
        
if __name__ == '__main__':
    pass
        