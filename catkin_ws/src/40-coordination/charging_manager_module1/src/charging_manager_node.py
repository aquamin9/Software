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



class ChargingManager(object):

    def __init__(self):
        # Hardcoded color values to configure traffic light
        # ATTENTION: This script uses GRB instead of RGB logic to work with the
        # newest version of the traffic lights with surface mounted LEDs.
        #TODO: For each frequency set the color differently
        self.green_color = [1, 0, 0]
        self.red_color = [0, 1, 0]
        self.yellow_color = [1, 1, 0]
        self.black_color = [0, 0, 0]
        self.blue_color = [0, 0, 1]
        self.purple_color = [0, 1, 1]
        #self.color = self.green_color
        self.led = RGB_LED()

        self.node_name = rospy.get_name()
        self.cycle = None
        #Initialize a point 
        self.initial_point = Point()
        self.setupParams()

        #FREQUENCIES FOR CHARGERS
        # f1 = 1.9 
        self.freq_CH1 = self.protocol['signals']['CAR_SIGNAL_A_OLD']['frequency']
        #f2 = 4
        self.freq_CH2 = self.protocol['signals']['CAR_SIGNAL_A']['frequency']

        #PERIODS FOR CHARGERS
        self.T_CH1 = 1.0/self.freq_CH1
        self.T_CH2 = 1.0/self.freq_CH2

        #List of lights' indices
        self.light_list = [0,2,3,4]

        #Light States
        self.light_state_dict = {0:False , 2:False, 3:False, 4:False}

        #Initializing the first frequency 
        self.charger_next_free = 2
        #CHARGER SIZES
        self.charger1_size = 0
        self.charger2_size = 0
        
        #Previous CHARGER SIZES
        self.charger1_size_old = self.charger1_size
        self.charger2_size_old = self.charger2_size
        
        self.charger1_full = False 
        self.charger2_full = False

        #'apriltag':timestamp_secs will be saved in dictionaries of chargers 
        self.chargers ={'charger1':{},'charger2':{}}

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
        self.timer_charging_manager = rospy.Timer(rospy.Duration(10.0),self.cbChargingManager)
        self.timer_CH = rospy.Timer(rospy.Duration(0.5*self.T_CH2),self.cbTimerCH)
        self.timer_updateChargerSizes=rospy.Timer(rospy.Duration(self.updateChargerSizeTime),self.updateChargerSizes)
        self.timer_debug = rospy.Timer(rospy.Duration(self.debug_time),self.DebugLists)






 #-----------------------------Charger Management START-----------------------------# 
    

    #For debug purposes
    def DebugLists(self,event):
        rospy.loginfo("###########################")
        rospy.loginfo("STATIC AT:"+ str(self.staticAprilTags))
        rospy.loginfo("MOVING AT:"+str(self.movingAprilTags))
        rospy.loginfo("Charging Manager: "+str(self.chargers))
        rospy.loginfo("Charging Manager: Charger 1 Occupancy: "+str(self.charger1_size)+", Charger 2 Occupancy: "+str(self.charger2_size))




    #Delete the tagID from self.movingAprilTags dictionary
    def deleteMovingBot(self,tagID):
        for botID in self.movingAprilTags.keys():
            if(tagID == botID):
                try:
                    del self.movingAprilTags[tagID]
                    rospy.loginfo("["+self.node_name+"] "+str(botID)+" is deleted from self.movingAprilTags")
                except KeyError:
                    rospy.loginfo("["+self.node_name+"] tagID "+ str(tagID)+" could not be found in "+str(self.movingAprilTags)+" movingAprilTags: "+str(self.movingAprilTags))
        
    #delete the duckiebot apriltag from self.chargers
    def releaseChargerSpot(self,tagID):
        #delete the tagID from self.chargers 
        for charger in self.chargers.keys():
            #List of bots in charger
            bots = list(self.chargers[charger].keys())

            #Delete the tagID from chargers dictionary
            if(tagID in bots):

                try:
                    del self.chargers[charger][tagID]
                    rospy.loginfo("Charging Manager: "+str(tagID)+" released charger "+str(charger))
                except KeyError :
                    rospy.loginfo("["+self.node_name+"] tagID "+ str(tagID)+" could not be found in "+str(charger)+". Charger has "+str(bots))

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
                    self.chargers['charger1'][botID] = self.movingAprilTags[botID]['timestamp']
                    self.deleteMovingBot(botID) 

                #FIRST:@entrance or @exit 
                #LAST:@direction2_tag
                elif(self.movingAprilTags[botID]['last_neighbor'] == self.direction2_tag) and (self.movingAprilTags[botID]['first_neighbor'] == self.entrance or self.movingAprilTags[botID]['first_neighbor'] == self.exit):
                    rospy.loginfo(str(botID)+" on WAY "+str(self.direction2))
                    self.chargers['charger2'][botID] = self.movingAprilTags[botID]['timestamp']
                    self.deleteMovingBot(botID)


                #FIRST:@direction1_tag or @direction2_tag
                #LAST:@entrance or @exit 
                elif (self.movingAprilTags[botID]['last_neighbor'] == self.entrance or self.movingAprilTags[botID]['last_neighbor'] == self.exit) and (self.movingAprilTags[botID]['first_neighbor'] == self.direction2_tag or self.movingAprilTags[botID]['first_neighbor'] == self.direction1_tag) :
                    rospy.loginfo(str(botID)+" on WAY EXIT")
                    self.releaseChargerSpot(botID)
                    self.deleteMovingBot(botID)
                
                #Some undesired states

                #FIRST:@entrance or @exit
                #LAST:@entrance or @exit 
                elif (self.movingAprilTags[botID]['last_neighbor'] == self.entrance or self.movingAprilTags[botID]['last_neighbor'] == self.exit) and (self.movingAprilTags[botID]['first_neighbor'] == self.entrance or self.movingAprilTags[botID]['first_neighbor'] == self.exit) : 
                    rospy.loginfo("["+self.node_name+"] The Tag ID "+str(botID)+" is seen near to the entrance or exit AprilTags in the first and last picture")
                    self.releaseChargerSpot(botID)
                    self.deleteMovingBot(botID)

                #FIRST = LAST
                elif(self.movingAprilTags[botID]['last_neighbor'] == self.movingAprilTags[botID]['first_neighbor']):
                    rospy.loginfo("["+self.node_name+"] The Duckiebot with AT "+str(botID)+" ")
                    rospy.loginfo("["+self.node_name+"] Check whether the ground april tags are well positioned.")
                    self.releaseChargerSpot(botID)
                    self.deleteMovingBot(botID)
                    
                #FIRST:@direction1_tag or direction2_tag
                #LAST:@direction1_tag or direction2_tag
                elif(self.movingAprilTags[botID]['first_neighbor'] == self.direction1_tag and self.movingAprilTags[botID]['last_neighbor'] == self.direction2_tag) or (self.movingAprilTags[botID]['first_neighbor'] == self.direction2_tag and self.movingAprilTags[botID]['last_neighbor'] == self.direction1_tag):
                    rospy.loginfo("["+self.node_name+"] Check whether the ground april tags are well positioned.")
                    rospy.loginfo("["+self.node_name+"] There might be an issue with the intersection control in duckiebot")
                    self.releaseChargerSpot(botID)
                    self.deleteMovingBot(botID)

                else : 
                    rospy.loginfo("["+self.node_name+"] Something unexpected has heppend in updateLastNeighbor")


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

                rospy.loginfo("["+self.node_name+"]Static AprilTags changed entrance: "+str(self.entrance)+" exit: "+str(self.exit)+" direction1: "+str(self.direction1_tag)+" direction2: "+str(self.direction2_tag)+" direction1:"+str(self.direction1)+" direction2: "+str(self.direction2))
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
    


    


    #Updates the amount of duckiebots in chargers by looking up the self.chargers dictionary
    def updateChargerSizes(self,event):
        for charger in self.chargers.keys():
            if(charger == 'charger1'):
                self.charger1_size = int(len(self.chargers[charger].keys()))

                if(self.charger1_size == self.charger1_capacity):
                    self.charger1_full = True
                elif( self.charger1_size < self.charger1_capacity):
                    self.charger1_full = False

            elif(charger == 'charger2'):
                self.charger2_size = int(len(self.chargers[charger].keys()))

                if(self.charger2_size == self.charger2_capacity):
                    self.charger2_full = True
                elif( self.charger2_size < self.charger2_capacity):
                    self.charger2_full = False
            else:
                rospy.loginfo("["+self.node_name+"]Something unexpected has happened in updateChargerSizes")

    #Set the charger_next_free according to the least occupied charger
    #If chargers are equally occupied, the charger with lowest index will be set as charger_next_free 
    def cbChargingManager(self, event):
        
        if((self.charger1_size != self.charger1_size_old) or (self.charger2_size != self.charger2_size_old)): #only allowing if charger_next_free was updated
            #Find the next free charger 
            self.charger_next_free = self.findNextFreeCharger()
        
            #blink the LED with a frequency according to the next free charger 
            if self.charger_next_free == 1 :
                self.timer_CH.shutdown()
                #self.color = self.red_color
                self.timer_CH = rospy.Timer(rospy.Duration(0.5*self.T_CH1),self.cbTimerCH)
            elif self.charger_next_free == 2 :
                self.timer_CH.shutdown()
                #self.color = self.green_color
                self.timer_CH = rospy.Timer(rospy.Duration(0.5*self.T_CH2),self.cbTimerCH)

            else : 
                rospy.loginfo("["+self.node_name+"]Something unexpected has happened in cbChargingManager")

            rospy.loginfo("Charging Manager: The new next free charger is "+str(self.charger_next_free))
            #set the charger size memories to the current value
            self.charger1_size_old = self.charger1_size
            self.charger2_size_old = self.charger2_size


    def findNextFreeCharger(self):
        #"Find the most unoccupied charger" This statement corresponds finding the minimal self.charger_size
        min_charger_size = self.charger1_size
        min_charger_index = 1
        #Charging Manager will send DBs to the chargers 1, 2, 3 and then 4 in order
        CH_sizes = [self.charger1_size,self.charger2_size]
        CH_full = [self.charger1_full,self.charger2_full]
        for i in range(0,len(CH_sizes)):
            if CH_sizes[i] < min_charger_size and not CH_full[i]:
                min_charger_size = CH_sizes[i] 
                min_charger_index = i+1

        return min_charger_index
        

    def cbTimerCH(self,event):
        for light_number in self.light_list:
            self.lightToggle(light_number,"red")
            #self.lightToggle(light_number)


    #def lightToggle(self,light_number)
    def lightToggle(self,light_number,light_color):
        

        if(self.light_state_dict[light_number] == True):
            self.led.setRGB(light_number, self.black_color)
            self.light_state_dict[light_number] = False
            #traffic light is switched off
        else:
            #self.led.setRGB(light_number, self.color)
            #TODO: change the code for multiple color 
            if(light_color == "red"):
                self.led.setRGB(light_number, self.red_color)
            else:
                self.led.setRGB(light_number, self.red_color)

            self.light_state_dict[light_number] = True
            #traffic light is switched on 
       
 #-----------------------------Charger Management END-----------------------------#   


    def setupParams(self):
        self.entrance = self.setupParameter("~entrance",'')
        self.exit = self.setupParameter("~exit",'')
        self.direction1_tag = self.setupParameter("~direction1_tag",'')
        self.direction2_tag = self.setupParameter("~direction2_tag",'') 
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
        self.charger1_capacity = self.setupParameter("~charger1_capacity",2)
        self.charger2_capacity = self.setupParameter("~charger2_capacity",2)
        self.protocol = self.setupParameter("~LED_protocol", [])


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
        self.debug_time = rospy.get_param("~debug_time")
        self.charger1_capacity = rospy.get_param("~charger1_capacity")
        self.charger2_capacity = rospy.get_param("~charger2_capacity")

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)


        # Write to parameter server for transparancy
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

if __name__ == '__main__':
    rospy.init_node('charging_manager_node', anonymous=False)
    node = ChargingManager()
    rospy.spin()
