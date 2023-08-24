# coding: utf8
#!/usr/bin/env python3
import Sofa
from std_msgs.msg import Float64MultiArray,Float64

def cable_displacement_recv(data, datafield):

    datafield.value = data.data
    

def end_effector_pos_send(data):
    end_effector_pos = Float64MultiArray()
    end_effector_pos.data = []
    data_list = []
    for value in data.value:
        data_list.append(float(value))
    
    end_effector_pos.data = data_list
   
    return end_effector_pos


class CableROSController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.cables = kwargs["cables"]
        self.robot = kwargs["robot"]
        self.rosNode = kwargs["rosNode"]
        self.robotToSim = kwargs["robotToSim"]
        self.receiving_topic = None
        self.index = 0
        self.Flag = None

    def get_index_by_rosNode(self): 
        
        topic_name_format = ["/Inverse/Cable1/state/displacement", "/Inverse/Cable2/state/displacement",
                             "/Inverse/Cable3/state/displacement", "/Inverse/Cable4/state/displacement",
                             "/Inverse/Cable5/state/displacement", "/Inverse/Cable6/state/displacement",
                             "/Inverse/Cable7/state/displacement","/Inverse/Cable8/state/displacement",
                             "/Inverse/Cable9/state/displacement"]
        #Get active topic name and type
        topics_names_and_types = self.rosNode.get_topic_names_and_types()

        for topic_name, topic_type in topics_names_and_types:
            #check the topic_type std_msgs/msg/Float64 to get corresponding topic_name
            if topic_type ==['std_msgs/msg/Float64'] and topic_name in topic_name_format:
                #get the list of publishers thanks to the get_publishers_info_by_topic method
                publishers = self.rosNode.get_publishers_info_by_topic(topic_name)
                if publishers:
                    self.Flag = False
                    # If publishers is True => whe have topic which is receiving message
                    self.receiving_topic = topic_name
                    topic_processing = self.receiving_topic.strip().split('/')
                    self.index = int(topic_processing[2].strip("Cable"))
                    break
            
                else:
                    self.Flag = True

        
        return self.Flag

    
    # action at beginning frame
    def onAnimateBeginEvent(self, event):
               
        state = self.get_index_by_rosNode()
        if not state:
            #If node is active and receiving messages, state is False
            with self.cables[self.index - 1].value.writeable() as t:
                t[0] +=  [self.robot.effector_cable_data.value]
        else:
            #If no message is received, Echelon3 keeps its last position
            with self.cables[self.index-1].value.writeable() as t:
                t[0] = t[0]

        #Get the postion of the end of the Echelon3
        with self.robot.end_effector_pos.writeable() as  end_effector_pos :
            for i in range(0,7):
                end_effector_pos[i] = self.robot.Chassis.AttachedArm.Trunk.framesNode.frames.position.value[100][i]