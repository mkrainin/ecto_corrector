#!/usr/bin/env python
import ecto, ecto_ros, ecto_pcl, ecto_pcl_ros, ecto_sensor_msgs, ecto_opencv.highgui
import ecto_test
import ecto_corrector
        
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
        