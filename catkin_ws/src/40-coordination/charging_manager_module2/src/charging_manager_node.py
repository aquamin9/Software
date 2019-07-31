#!/usr/bin/env python
import rospy
from rgb_led import *
import sys
import time
from std_msgs.msg import Float32, Int8, Int32, Bool
from geometry_msgs.msg import Point
from duckietown_msgs.msg import AprilTagDetection
from rgb_led import RGB_LED
from std_msgs.msg import String
#for nested dictionary initialization
from collections import defaultdict
from duckietown_utils import tcp_communication 


# COLORS and FREQUENCIES of lights
#frequencies 
#f3 = 5.7
#f4=7.8
#f5=10.6
#f2=2.4
#f1=1.9
# purplelight : (CAR_SIGNAL_B):f2 and purple
# greenlight  : f4 and green
# redlight :  (traffic_light_stop) f3 and green
# yellowlight 	  : (CAR_SIGNAL_C):f5 and yellow


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
        #self.node_name = rospy.get_name()
        self.node_name = "Charging Manager"
        self.cycle = None
        self.setupParams()

        
         

        #FREQUENCIES FOR CHARGERS
        # f1 = 1.9 
        self.freq_CH1 = self.protocol['signals']['CAR_SIGNAL_A_OLD']['frequency']
        #f2 = 4
        self.freq_CH2 = self.protocol['signals']['CAR_SIGNAL_A']['frequency']
        #f3 = 5.7
        self.freq_CH3 =self.protocol['signals']['CAR_SIGNAL_GREEN']['frequency']
        #f4 = 7.8
        self.freq_CH4 = self.protocol['signals']['traffic_light_go']['frequency']

        #PERIODS FOR CHARGERS
        self.T_CH1 = 1.0/self.freq_CH1
        self.T_CH2 = 1.0/self.freq_CH2
        self.T_CH3 = 1.0/self.freq_CH3 
        self.T_CH4 = 1.0/self.freq_CH4


        #List of lights' indices
        self.light_list = [0,2,3,4]

        #Light States
        self.light_state_dict = {0:False , 2:False, 3:False, 4:False}

        #CHARGER SIZES
        self.charger1_size = 0
        self.charger2_size = 0
        self.charger3_size = 0
        self.charger4_size = 0

        #Previous CHARGER SIZES
        self.charger1_size_old = self.charger1_size
        self.charger2_size_old = self.charger2_size
        self.charger3_size_old = self.charger3_size
        self.charger4_size_old = self.charger4_size

        #Initializing the first frequency 
        self.charger_next_free = 2

        self.charger1_full = False 
        self.charger2_full = False
        self.charger3_full = False
        self.charger4_full = False

        self.bots_in_chargers = []

        #'apriltag':timestamp_secs will be saved in dictionaries of chargers 
        self.chargers ={'charger1':{},'charger2':{},'charger3':{},'charger4':{}}
        
        
        #TIMERS 
        self.timer_updateParams = rospy.Timer(rospy.Duration(1.0),self.updateParams)
        self.timer_charging_manager = rospy.Timer(rospy.Duration(5.0),self.cbChargingManager)
        self.timer_CH = rospy.Timer(rospy.Duration(0.5*self.T_CH2),self.cbTimerCH)
        self.timer_updateChargerSizes=rospy.Timer(rospy.Duration(self.updateChargerSizeTime),self.updateChargerSizes)
        self.debug = rospy.Timer(rospy.Duration(self.debug_time),self.DebugLists)
        self.timer_duckiebots = rospy.Timer(rospy.Duration(2.0),self.cbTimerDuckiebots)



    #Set the charger_next_free according to the least occupied charger
    #If chargers are equally occupied, the charger with lowest index will be set as charger_next_free 
    def cbChargingManager(self, event):
        
        if((self.charger1_size != self.charger1_size_old) or (self.charger2_size != self.charger2_size_old) or (self.charger3_size != self.charger3_size_old) or (self.charger4_size != self.charger4_size_old)): #only allowing if charger_next_free was updated
            #Find the next free charger 
            self.charger_next_free = self.findNextFreeCharger()
            #rospy.loginfo("#######Charger sizes changed########")
        
            #blink the LED with a frequency according to the next free charger 
            if self.charger_next_free == 1 :
                self.timer_CH.shutdown()
                #self.color = self.red_color
                self.timer_CH = rospy.Timer(rospy.Duration(0.5*self.T_CH1),self.cbTimerCH)
            elif self.charger_next_free == 2 :
                self.timer_CH.shutdown()
                #self.color = self.green_color
                self.timer_CH = rospy.Timer(rospy.Duration(0.5*self.T_CH2),self.cbTimerCH)
            elif self.charger_next_free == 3 :
                self.timer_CH.shutdown()
                #self.color = self.blue_color
                self.timer_CH = rospy.Timer(rospy.Duration(0.5*self.T_CH3),self.cbTimerCH)
            elif self.charger_next_free == 4 :
                self.timer_CH.shutdown()
                #self.color = self.yellow_color
                self.timer_CH = rospy.Timer(rospy.Duration(0.5*self.T_CH4),self.cbTimerCH)
            else : 
                rospy.loginfo("["+self.node_name+"]Something unexpected has happened in cbChargingManager")

            rospy.loginfo("["+self.node_name+"] The new next free charger is "+str(self.charger_next_free))
            #set the charger size memories to the current value
            self.charger1_size_old = self.charger1_size
            self.charger2_size_old = self.charger2_size
            self.charger3_size_old = self.charger3_size
            self.charger4_size_old = self.charger4_size 

    def findNextFreeCharger(self):
        #"Find the most unoccupied charger" This statement corresponds finding the minimal self.charger_size
        min_charger_size = self.charger1_size
        min_charger_index = 1
        #Charging Manager will send DBs to the chargers 1, 2, 3 and then 4 in order
        CH_sizes = [self.charger1_size,self.charger2_size,self.charger3_size,self.charger4_size]
        CH_full = [self.charger1_full,self.charger2_full, self.charger3_full,self.charger4_full]
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



    #Fetch the data about duckiebot:charger from TCP server 
    def cbTimerDuckiebots(self,event):
        self.bots_in_chargers = []
        #Get the bots in chargers from the YAML file which is served to the others by TCP communication
        for ch in self.chargers.keys():
            self.bots_in_chargers.extend(self.chargers[ch].keys())
        

        for bot, charger in self.duckiebots.items():
            if (charger != 0) and ( str(bot) not in self.bots_in_chargers ) :
                #rospy.loginfo("Charging Manager: Duckiebot "+str(bot)+" has entered charger "+str(charger))
                current_time = rospy.get_rostime().to_sec()
                self.chargers["charger"+str(charger)][str(bot)] = current_time
            elif charger == 0 and str(bot) in self.bots_in_chargers :
                self.releaseChargerSpot(bot)

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
                    rospy.loginfo("["+self.node_name+"] "+str(tagID)+" released charger "+str(charger))
                except KeyError :
                    rospy.loginfo("["+self.node_name+"] tagID "+ str(tagID)+" could not be found in "+str(charger)+". Charger has "+str(bots))


    def DebugLists(self,event):
        rospy.loginfo("###########################")
        rospy.loginfo("["+self.node_name+"] "+str(self.chargers))
        #rospy.loginfo("["+self.node_name+"] Charger 1 Occupancy: "+str(self.charger1_size)+", Charger 2 Occupancy: "+str(self.charger2_size)+", Charger 3 Occupancy: "+str(self.charger3_size)+", Charger 4 Occupancy: "+str(self.charger4_size))


    #Updates the amount of duckiebots in chargers by looking up the self.chargers dictionary
    def updateChargerSizes(self,event):
        #rospy.loginfo("update started")
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

            elif(charger == 'charger3'):
                self.charger3_size = int(len(self.chargers[charger].keys()))

                if(self.charger3_size == self.charger3_capacity):
                    self.charger3_full = True
                elif( self.charger3_size < self.charger3_capacity):
                    self.charger3_full = False

            elif(charger == 'charger4'):
                self.charger4_size = int(len(self.chargers[charger].keys()))

                if(self.charger4_size == self.charger4_capacity):
                    self.charger4_full = True
                elif( self.charger4_size < self.charger4_capacity):
                    self.charger4_full = False


            else:
                rospy.loginfo("["+self.node_name+"]Something unexpected has happened in updateChargerSizes")
        #rospy.loginfo("update ended")





    



    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparancy
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value        
    def setupParams(self):
        #periods of timers
        self.updateChargerSizeTime = self.setupParameter("~updateChargerSizeTime",1.0)
        self.threshold = self.setupParameter("~threshold",1.0)
        self.bookkeeping_time = self.setupParameter("~bookkeeping_time",1.0)
        self.debug_time = self.setupParameter("~debug_time",1.0)
        #Datasheet of duckietown frequencies
        self.protocol = self.setupParameter("~LED_protocol", [])
        #Duckiebot Apriltags 
        self.duckiebots = self.setupParameter("~duckiebots", [])
        #CHARGER CAPACITIES
        self.charger1_capacity = self.setupParameter("~charger1_capacity",2)
        self.charger2_capacity = self.setupParameter("~charger2_capacity",2)
        self.charger3_capacity = self.setupParameter("~charger3_capacity",2)
        self.charger4_capacity = self.setupParameter("~charger4_capacity",2) 

    def updateParams(self,event):

        self.updateChargerSizeTime = rospy.get_param("~updateChargerSizeTime")
        self.threshold = rospy.get_param("~threshold")
        self.bookkeeping_time = rospy.get_param("~bookkeeping_time")
        self.duckiebots = rospy.get_param("/tcp_server/tcp_communication_server_node/duckiebots")
        self.charger1_capacity = rospy.get_param("~charger1_capacity")
        self.charger2_capacity = rospy.get_param("~charger2_capacity")
        self.charger3_capacity = rospy.get_param("~charger3_capacity")
        self.charger4_capacity = rospy.get_param("~charger4_capacity")



if __name__ == '__main__':
    rospy.init_node('charging_manager', anonymous=False)
    node = ChargingManager()
    rospy.spin()
