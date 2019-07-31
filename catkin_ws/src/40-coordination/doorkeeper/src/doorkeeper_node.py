#!/usr/bin/env python
import rospy
from rgb_led import *
import sys
import time
from std_msgs.msg import Float32, Int8, Int32, Bool, String
from geometry_msgs.msg import Point
from duckietown_msgs.msg import AprilTagDetection
from rgb_led import RGB_LED
from std_msgs.msg import String
#for nested dictionary initialization
from collections import defaultdict
from duckietown_utils import tcp_communication 



class Doorkeeper(object):

    def __init__(self):
        
        self.node_name = rospy.get_name()
        self.cycle = None
        #Initialize a point 
        self.initial_point = Point()
        self.setupParams()

        #Get the static April Tags (right now it is hardcoded)
        #keys : apriltag ID values: positions 
        #TODO: Find a way to get the tag ids automaticaly via a YAML file...etc
        self.staticAprilTags = {self.entrance:{'position':self.initial_point},\
        self.direction1_tag:{'position':self.initial_point},\
        self.direction2_tag:{'position':self.initial_point},\
        self.exit:{'position':self.initial_point}}
        
        #Save the moving April Tags as {'tagID':{'position':Point,'neighbor':tagIDNeighbor,'timestamp':time,'direction':directions}}
        #this dictionary will be updated periodicaly 
        self.movingAprilTags = defaultdict(dict)
 
        #SUBSCRIBERS
        self.sub_poses = rospy.Subscriber("/poses_acquisition/poses",AprilTagDetection,self.cbPoses)
        
        self.timer_last_neighbor = rospy.Timer(rospy.Duration(self.bookkeeping_time),self.updateLastNeighbor)
        self.timer_updateParams = rospy.Timer(rospy.Duration(1.0),self.updateParams)
        self.debug_timer = rospy.Timer(rospy.Duration(self.debug_time),self.DebugLists)





 #-----------------------------Charger Management START-----------------------------# 
    

    #For debug purposes
    def DebugLists(self,event):
        rospy.loginfo("###########################")
        rospy.loginfo("STATIC AT:"+ str(self.staticAprilTags))
        rospy.loginfo("MOVING AT:"+str(self.movingAprilTags))






    #Delete the tagID from self.movingAprilTags dictionary
    def deleteMovingBot(self,tagID):
        for botID in self.movingAprilTags.keys():
            if(tagID == botID):
                try:
                    del self.movingAprilTags[tagID]
                    rospy.loginfo("["+self.node_name+"] "+str(botID)+" is deleted from self.movingAprilTags")
                except KeyError:
                    rospy.loginfo("["+self.node_name+"] tagID "+ str(tagID)+" could not be found in "+str(self.movingAprilTags)+" movingAprilTags: "+str(self.movingAprilTags))
        


    def updateLastNeighbor(self,event):
        current_time = rospy.get_rostime().to_sec()


        for botID in self.movingAprilTags.keys():
            #If the last time stamp is not updated for a long time, take action according to the last neighbor
            rospy.loginfo(str(abs(current_time-self.movingAprilTags[botID]['timestamp']))+ " time difference "+ str(botID))

            if(abs(current_time-self.movingAprilTags[botID]['timestamp'])> self.threshold):

                #FIRST:@entrance or @exit 
                #LAST:@direction1_tag
                if (self.movingAprilTags[botID]['last_neighbor'] == self.direction1_tag) and (self.movingAprilTags[botID]['first_neighbor'] == self.entrance or self.movingAprilTags[botID]['first_neighbor'] == self.exit):
                    rospy.loginfo(str(botID)+" on WAY "+str(self.direction1))
                    self.sendToServer(str(botID),self.direction1)
                    self.deleteMovingBot(botID) 

                #FIRST:@entrance or @exit 
                #LAST:@direction2_tag
                elif(self.movingAprilTags[botID]['last_neighbor'] == self.direction2_tag) and (self.movingAprilTags[botID]['first_neighbor'] == self.entrance or self.movingAprilTags[botID]['first_neighbor'] == self.exit):
                    rospy.loginfo(str(botID)+" on WAY "+str(self.direction2))
                    self.sendToServer(str(botID),self.direction2)
                    self.deleteMovingBot(botID)


                #FIRST:@direction1_tag or @direction2_tag
                #LAST:@entrance or @exit 
                elif (self.movingAprilTags[botID]['last_neighbor'] == self.entrance or self.movingAprilTags[botID]['last_neighbor'] == self.exit) and (self.movingAprilTags[botID]['first_neighbor'] == self.direction2_tag or self.movingAprilTags[botID]['first_neighbor'] == self.direction1_tag) :
                    rospy.loginfo(str(botID)+" on WAY EXIT")
                    self.sendToServer(str(botID),0)
                    self.deleteMovingBot(botID)
                
                #Some undesired states

                #FIRST:@entrance or @exit
                #LAST:@entrance or @exit 
                elif (self.movingAprilTags[botID]['last_neighbor'] == self.entrance or self.movingAprilTags[botID]['last_neighbor'] == self.exit) and (self.movingAprilTags[botID]['first_neighbor'] == self.entrance or self.movingAprilTags[botID]['first_neighbor'] == self.exit) : 
                    rospy.loginfo("["+self.node_name+"] The Tag ID "+str(botID)+" is seen near to the entrance or exit AprilTags in the first and last picture")
                    self.sendToServer(str(botID),0)
                    self.deleteMovingBot(botID)

                #FIRST = LAST
                elif(self.movingAprilTags[botID]['last_neighbor'] == self.movingAprilTags[botID]['first_neighbor']):
                    rospy.loginfo("["+self.node_name+"] The Duckiebot with AT "+str(botID)+" ")
                    rospy.loginfo("["+self.node_name+"] Check whether the ground april tags are well positioned.")
                    self.sendToServer(str(botID),0)
                    self.deleteMovingBot(botID)
                    
                #FIRST:@direction1_tag or direction2_tag
                #LAST:@direction1_tag or direction2_tag
                elif(self.movingAprilTags[botID]['first_neighbor'] == self.direction1_tag and self.movingAprilTags[botID]['last_neighbor'] == self.direction2_tag) or (self.movingAprilTags[botID]['first_neighbor'] == self.direction2_tag and self.movingAprilTags[botID]['last_neighbor'] == self.direction1_tag):
                    rospy.loginfo("["+self.node_name+"] Check whether the ground april tags are well positioned.")
                    rospy.loginfo("["+self.node_name+"] There might be an issue with the intersection control in duckiebot")
                    self.sendToServer(str(botID),0)
                    self.deleteMovingBot(botID)

                else : 
                    rospy.loginfo("["+self.node_name+"] Something unexpected has heppend in updateLastNeighbor")





    
    
    def sendToServer(self,tagID,charger_to_set):
        bots = tcp_communication.getVariable("duckiebots")
        rospy.loginfo("["+self.node_name+"] List of bots:"+str(bots))
        #If the tagID was not assigned to a charger, assign a charger to him and send it to server 
        if bots[tagID] == 0:
            rospy.loginfo("["+self.node_name+"] Bots charger was in initial state: 0")
            success = tcp_communication.setVariable("duckiebots/" + str(tagID) , charger_to_set)
            rospy.loginfo("["+self.node_name+"] Setting bot's charger to:"+str(charger_to_set)) 
            #Check whether the value is set correctly
            if(success):
                #Backoff a little bit for precaution
                rospy.sleep(3.0)
                charger_now = tcp_communication.getVariable("duckiebots/"+str(tagID))
                if(charger_to_set == charger_now):
                    rospy.loginfo("["+self.node_name+"] The Charger for "+str(tagID)+" is set correctly")
                
            elif(sucess =="ERROR"):
                rospy.loginfo("["+self.node_name+"] Value was too long to fit inside the BUFFER_SIZE")
            else:
                rospy.loginfo("["+self.node_name+"]Charger for "+str(tagID)+" "+str(charg)+" could not be set!")
        #If the tagID was assigned to a charger, then set its charger to 0          
        else : 
            rospy.loginfo("["+self.node_name+"] Bots charger was in  state: "+str(bots[tagID]))
            success = tcp_communication.setVariable("duckiebots/" + str(tagID) , charger_to_set)
            rospy.loginfo("["+self.node_name+"] Setting bot's charger to:"+str(charger_to_set)) 
            #Check Whether the value is set correctly
            if(success):
                #Backoff a little bit for precaution
                rospy.sleep(3.0)
                charger_now = tcp_communication.getVariable("duckiebots/"+str(tagID))
                if(charger_to_set == charger_now):
                    rospy.loginfo("["+self.node_name+"] The Charger for "+str(tagID)+" is set correctly")
                
            elif(sucess =="ERROR"):
                rospy.loginfo("["+self.node_name+"] Value was too long to fit insde the BUFFER_SIZE")
            else:
                rospy.loginfo("["+self.node_name+"]Charger for "+str(tagID)+" "+str(charg)+" could not be set!")

        
    def isDuckieBot(self,tagID):
        #look up whether the april tag is owned by a duckiebot 
        if(tagID in self.VehicleTags):
            return True
        else:
            return False


    #Finds the neighbor april tag to the given moving april tag
    def NearestNeighbor(self,tagPose):
        nearestTag = ''
        nearestPosition = 16000000
        #{'tagID':{'position':Point,'neighbor':tagIDNeighbor,'timestamp':time,'charger':chargerID}}
        for tagIDNeighbor, Neighbor in self.staticAprilTags.items():
            #absolute value of distance between a static tag and amoving tag squared 
            d = (Neighbor['position'].x -tagPose.x)**2 + (Neighbor['position'].y -tagPose.y)**2 
            if(d < nearestPosition):
                nearestPosition = d
                nearestTag = tagIDNeighbor
        if(len(nearestTag) == 0):
            rospy.loginfo("["+self.node_name+"]No nearest neighbor can be found.")
        return nearestTag




    def cbPoses(self,msg):
        #rospy.loginfo("["+self.node_name+"]"+" cbPoses message arrived: TagId: "+str(msg.tag_id)+" center: "+str(msg.center))
        current_time = rospy.get_rostime().to_sec()

        tagID = str(msg.tag_id)
        #3D Point just using x and y to save the pixels of the center of an april tag
        tagPose = Point()
        tagPose.x = msg.center[0] 
        tagPose.y = msg.center[1]
        tagPose.z = 0

        for tag in [str(self.entrance), str(self.direction1_tag), str(self.direction2_tag), str(self.exit)]: 
            if tag in list(self.staticAprilTags.keys()): 
                continue
            else:

                rospy.loginfo("["+self.node_name+"]Static AprilTags changed entrance: "+str(self.entrance)+" exit: "+str(self.exit)+" direction1_tag: "+str(self.direction1_tag)+" direction2_tag: "+str(self.direction2_tag)+" direction1:"+str(self.direction1)+" direction2: "+str(self.direction2))
                self.entrance = str(self.entrance)
                self.exit = str(self.exit)
                self.direction1_tag = str(self.direction1_tag)
                self.direction2_tag = str(self.direction2_tag)
                self.staticAprilTags = {self.entrance:{'position':self.initial_point},\
                    self.direction1_tag:{'position':self.initial_point},\
                    self.direction2_tag:{'position':self.initial_point},\
                    self.exit:{'position':self.initial_point}}
                break

        if(tagID in self.staticAprilTags.keys()):
            self.staticAprilTags[tagID]['position'] = tagPose
             
        
        if(self.isDuckieBot(tagID)):
            if tagID not in list(self.movingAprilTags.keys()) :
                self.movingAprilTags[tagID]['first_neighbor'] = self.NearestNeighbor(tagPose)


            self.movingAprilTags[tagID]['position'] = tagPose
            self.movingAprilTags[tagID]['timestamp'] = current_time
            self.movingAprilTags[tagID]['last_neighbor'] = self.NearestNeighbor(tagPose)

            #rospy.loginfo("["+self.node_name+"]"+"Duckiebot "+tagID+" is near to "+self.movingAprilTags[tagID]['neighbor'])

       
 #-----------------------------Charger Management END-----------------------------#   


    def setupParams(self):
        self.entrance = str(self.setupParameter("~entrance",''))
        self.exit = str(self.setupParameter("~exit",''))
        self.direction1_tag = str(self.setupParameter("~direction1_tag",''))
        self.direction2_tag = str(self.setupParameter("~direction2_tag",'')) 
        self.direction1 = self.setupParameter("~direction1",0)
        self.direction2 = self.setupParameter("~direction2",0)
        #TODO:get apriltagsDB.yaml from the launch file 
        #self.AprilTags = self.setupParameter("~apriltagsDB", [])
        #self.VehicleTags =[str(obj['tag_id']) for obj in self.AprilTags if obj['tag_type'] == 'Vehicle']
        self.VehicleTags = [str(x) for x in range(400,500)]
        self.updateChargerSizeTime = self.setupParameter("~updateChargerSizeTime",1.0)
        self.threshold = self.setupParameter("~threshold",1.0)
        self.bookkeeping_time = self.setupParameter("~bookkeeping_time",1.0)
        self.debug_time = self.setupParameter("~debug_time",1.0)

    def updateParams(self,event):
        #self.AprilTags = self.setupParameter("~apriltagsDB", [])
        #self.VehicleTags =[str(obj['tag_id']) for obj in self.AprilTags if obj['tag_type'] == 'Vehicle']
        self.entrance = str(rospy.get_param("~entrance"))
        self.exit = str(rospy.get_param("~exit"))
        self.direction1_tag = str(rospy.get_param("~direction1_tag"))
        self.direction2_tag = str(rospy.get_param("~direction2_tag")) 
        self.direction1 = rospy.get_param("~direction1")
        self.direction2 = rospy.get_param("~direction2")
        self.updateChargerSizeTime = rospy.get_param("~updateChargerSizeTime")
        self.threshold = rospy.get_param("~threshold")
        self.bookkeeping_time = rospy.get_param("~bookkeeping_time")

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)


        # Write to parameter server for transparancy
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

if __name__ == '__main__':
    rospy.init_node('doorkeeper_node', anonymous=False)
    node = Doorkeeper()
    rospy.spin()
