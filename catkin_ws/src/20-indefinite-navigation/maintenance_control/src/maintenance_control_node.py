#!/usr/bin/env python
import rospy
import numpy as np
from rgb_led import *
from rgb_led import RGB_LED
from duckietown_msgs.msg import SegmentList, Segment, TagInfo, BoolStamped, StopLineReading, LanePose, FSMState, AprilTagsWithInfos, TurnIDandType, MaintenanceState, VehiclePose
from std_msgs.msg import Float32, Int16, Bool, String
from geometry_msgs.msg import Point
import time
import math

class MaintenanceControlNode(object):
    def __init__(self):
        self.node_name = "Maintenance Control Node"

        ## setup Parameters
        self.setupParams()

        ## Internal variables
        self.maintenance_state = "NONE"
        self.active_navigation = False
        self.state = "JOYSTICK_CONTROL"
        self.calibration = False
        self.TL_detect = False
        #self.at_MT_intersection= False
        #TODO:for maintnance intersection part important
        self.detection_completed = False
        self.at_stop_line = False
        #For maintenance traffic lights 
        self.current_tag_id = '1'
        self.vehicle_seen = False

        self.tl_freq = 0.0
        self.tl_freq_before = 0.0
        self.votes = [0,0,0,0]

        self.led = RGB_LED()

        #TagInfo() contains the information on sign 
        self.info = TagInfo()
        
        ## Publishers
        self.pub_maintenance_state = rospy.Publisher("~maintenance_state", MaintenanceState, queue_size=1)
        self.pub_in_charger = rospy.Publisher("~in_charger", BoolStamped, queue_size=1)
        self.pub_turn_type = rospy.Publisher("~turn_id_and_type_out", TurnIDandType, queue_size=1)
        self.pub_stop_tag_seen = rospy.Publisher("~charging_stop_tag_seen", BoolStamped,queue_size = 1)
        self.pub_not_stop_tag_seen = rospy.Publisher("~not_charging_stop_tag_seen", BoolStamped,queue_size = 1)
        self.pub_not_veh_seen = rospy.Publisher("~not_vehicle_seen", BoolStamped,queue_size = 1)
        self.pub_at_MT_intersection = rospy.Publisher("~at_MT_intersection", BoolStamped,queue_size = 1)
        self.pub_timer_off = rospy.Publisher("~timer_off", BoolStamped, queue_size = 1)

        ## Subscribers
        self.sub_inters_done_detailed = rospy.Subscriber("~intersection_done_detailed", TurnIDandType, self.cbIntersecDoneDetailed)
        self.go_mt_charging = rospy.Subscriber("~go_mt_charging", Bool, self.cbGoMTCharging)
        self.go_mt_full = rospy.Subscriber("~go_mt_full", Bool, self.cbGoMTFull)
        self.calib_done = rospy.Subscriber("~calibration_done", Bool, self.cbCalibrationDone)
        self.sub_turn_type = rospy.Subscriber("~turn_id_and_type", TurnIDandType, self.cbTurnType)
        self.sub_apriltags_out = rospy.Subscriber("~apriltags_out",AprilTagsWithInfos,self.cbAprilTagsOut)
        self.set_state = rospy.Subscriber("~set_state", String, self.cbSetState)
        self.rdy_at_exit = rospy.Subscriber("~ready_at_exit",  BoolStamped, self.cbReadyAtExit)
        self.sub_trafficlight_frequency = rospy.Subscriber("~trafficlight_frequency", Float32, self.cb_trafficlight_frequency)
        self.sub_fsm_state = rospy.Subscriber("~fsm_state", FSMState, self.cbFSMState)
        self.sub_vehicle_detection = rospy.Subscriber("~vehicle_detected", BoolStamped, self.cbVehicleSeen)



        
        ## update Parameters timer
        self.params_update = rospy.Timer(rospy.Duration.from_sec(5.0), self.updateParams)
        #self.timer_at_MT = rospy.Timer(rospy.Duration.from_sec(0.5),self.cbAtMTIntersection)

    ##### Begin callback functions #####
    '''
    def cbAtMTIntersection(self,event):
        if self.at_MT_intersection : 
            self.pub_at_MT_intersection.publish(data = True)
        else: 
            self.pub_at_MT_intersection.publish(data = False)
    '''

    #Get the info on vehicle detection 
    def cbVehicleSeen(self,msg):
        self.vehicle_seen = msg.data
        not_veh_seen_msg = BoolStamped()
        not_veh_seen_msg.data = not self.vehicle_seen
        self.pub_not_veh_seen.publish(not_veh_seen_msg)

    #Get FSM State
    def cbFSMState(self,msg):
        self.state = msg.state
        if self.state == "WAIT" :
            for i in range(0,5) : 
                self.led.setRGB(i,[0,0,0])

            self.TL_detect = True
            rospy.loginfo("["+self.node_name+"] Waiting for 15 seconds") 
            rospy.sleep(15.0)
            self.TL_detect = False
            self.detection_completed = True

            #TODO: count the votes and set the charger accordingly and 
            selected_charger = int(np.argmax(self.votes)+1)
            rospy.set_param("/maintenance_charger", selected_charger)
            self.charger = selected_charger

            #turn_type_and_id_msg = TurnIDandType()
            #turn_type = self.getTurnType(self.charger, self.current_tag_id, self.maintenance_state)
            #turn_type_and_id_msg.turn_type = turn_type
            #turn_type_and_id_msg.tag_id = self.current_tag_id
            #rospy.loginfo("["+self.node_name+"]CHARGER: "+str(self.charger)+" TURN: "+str(turn_type)+" VOTES:"+str(self.votes))
            #self.pub_turn_type.publish(turn_type_and_id_msg)
            #self.at_MT_intersection = False
            #self.pub_at_MT_intersection.publish(data = False)
            #rospy.loginfo("["+self.node_name+"] VOTES: "+str(self.votes))
            self.votes = [0,0,0,0]
            rospy.loginfo("["+self.node_name+"] Waiting is over!") 
            rospy.loginfo("["+self.node_name+"]According to the Charging Manager I should go to charger "+str(selected_charger))
            self.pub_timer_off.publish(data = True)




        else : 
            self.pub_timer_off.publish(data = False)
            if self.maintenance_state == "NONE" or self.maintenance_state == "WAY_TO_MAINTENANCE" : 
                self.detection_completed = False



    
    # Executed as soon as Duckiebot leaves charging rails
    def cbReadyAtExit(self, msg):
        if self.maintenance_state == "CHARGING" and msg.data:
            new_state = "WAY_TO_CALIBRATING" if self.calibration else "WAY_TO_CITY"
            self.active_navigation = True
            self.changeMTState(new_state)

    
    # Checks whether the charger stop sign at the end of the charger rails is seen 
    def cbAprilTagsOut(self,msg):
        tag_info = msg.infos
        tag_detections = []
        for tag in tag_info: 
            
            #rospy.loginfo("["+self.node_name+"]tag_id: "+str(tag.id)+" maintenance_intersection: "+str(self.maintenance_intersection)+" detection_compl: "+str(self.detection_completed))
            '''
            if tag.id == self.maintenance_intersection['tag'] and (not self.detection_completed):
                #rospy.loginfo("["+self.node_name+"]#############WE ARE AT MAINTENANCE INTERSECTION#############")
                self.pub_at_MT_intersection.publish(data = True)
                #self.at_MT_intersection = True
            else :
                self.pub_at_MT_intersection.publish(data = False)
                #self.at_MT_intersection = False
            '''
            if tag.tag_type == self.info.SIGN and tag.traffic_sign_type == self.info.STOP : 
                tag_detections.append(tag.id)


        charger_stop_tag_seen = False 

        for ref_tag in self.charging_stop_signs['tag'] : 
            if charger_stop_tag_seen == True :
                break 
            elif ref_tag in tag_detections :
                charger_stop_tag_seen = True

        stop_tag_seen_msg = BoolStamped()
        stop_tag_seen_msg.data = charger_stop_tag_seen 
        self.pub_stop_tag_seen.publish(stop_tag_seen_msg)



        

    # For manual debugging
    def cbSetState(self, msg):
        self.changeMTState(msg.data)
        self.active_navigation = self.maintenance_state in ["WAY_TO_CHARGING", "WAY_TO_CALIBRATING", "WAY_TO_CITY", "WAY_TO_MAINTENANCE"]

    # Request to go charging only
    def cbGoMTCharging(self, msg):
        if msg.data:
            self.calibration = False
            self.changeMTState("WAY_TO_MAINTENANCE")

    # Request to do full maintenance (charging and calibration)
    def cbGoMTFull(self, msg):
        if msg.data:
            self.calibration = True
            self.changeMTState("WAY_TO_MAINTENANCE")

    # Called as soon as calibration has finished
    def cbCalibrationDone(self, msg):
        if msg.data:
            self.changeMTState("WAY_TO_CITY")
            self.active_navigation = True

    # Executes when intersection is done - this function is used to determine
    # if a maintenance state should be changed by driving into a specific area
    def cbIntersecDoneDetailed(self, msg):

        tag_id = msg.tag_id
        turn_type = msg.turn_type

        # Check if we just drove through an entrance defined in YAML file
        mt_entered = self.isInDict(tag_id, turn_type, self.maintenance_entrance)
        #self.MT_intersection_passed = (tag_id == self.maintenance_intersection)
        # Check if we just drove through an exit defined in YAML file
        mt_exited = self.isInDict(tag_id, turn_type, self.maintenance_exit)

        # Same scheme as above
        charging_entered = self.isInDict(tag_id, turn_type, self.stations['entrances'])
        charging_exited = self.isInDict(tag_id, turn_type, self.stations['exits'])

        calib_entered = self.isInDict(tag_id, turn_type,self.calibration_station['entrances'])
        calib_exited = self.isInDict(tag_id, turn_type, self.calibration_station['exits'])

        # summarize gates in two lists
        gate_bools = [mt_entered,           mt_exited,  charging_entered,   calib_entered]
        gate_trans = ["WAY_TO_CHARGING",    "NONE",     "CHARGING",         "CALIBRATING"]

        # Change state if Duckiebot drives through any gate
        for i in range(0, len(gate_bools)):
            if gate_bools[i]: self.changeMTState(gate_trans[i])

        # Notify world that we're in a charger
        if charging_entered:
            entered_msg = BoolStamped()
            entered_msg.data = True
            self.pub_in_charger.publish(entered_msg)

        # Turn on active navigation for WAY_TO_ states
        self.active_navigation = self.maintenance_state in ["WAY_TO_CHARGING", "WAY_TO_CALIBRATING", "WAY_TO_CITY", "WAY_TO_MAINTENANCE"]
    
    #Get the trafficlight frequency from the LED_detector node 
    def cb_trafficlight_frequency(self,msg) : 
        #rospy.loginfo("["+self.node_name+"] Traffic Light Frequency: "+str(msg.data))
        #Rounding 
        self.tl_freq = round(msg.data,1)

        if self.TL_detect:
            for i in range(0,4) : 
                if self.tl_freq == self.charger_frequencies[i] : 
                    self.votes[i] = self.votes[i] + 1
        

    # Adjust turn type if sign has a known ID for our path to charger
    def cbTurnType(self, msg):
        tag_id = msg.tag_id
        turn_type = msg.turn_type
        self.current_tag_id = tag_id
        
        if self.active_navigation:
            #The trafficlight frequency detection will be only interpreted when the Duckiebot arrives to the maintenance intersection 
            #and its FSM State is INTERSECTION_COORDINATION
            '''
            if self.current_tag_id == self.maintenance_intersection['tag'] and self.tl_freq != self.tl_freq_before and self.state == "WAIT":
                rospy.loginfo("["+self.node_name+"] I am at maintenance intersection, detected frequency ="+str(self.tl_freq))
                if self.tl_freq == self.charger_frequencies[0] :
                    rospy.set_param("/maintenance_charger", 1)
                    self.charger = 1
                elif self.tl_freq == self.charger_frequencies[1] :
                    rospy.set_param("/maintenance_charger", 2)
                    self.charger = 2
                elif self.tl_freq == self.charger_frequencies[2]:
                    rospy.set_param("/maintenance_charger", 3)
                    self.charger = 3
                elif self.tl_freq == self.charger_frequencies[3] :
                    rospy.set_param("/maintenance_charger", 4)
                    self.charger = 4

                else :
                    rospy.loginfo("[" + self.node_name + "]" + " Unexpected Frequency Detected: " + str(self.tl_freq) )
                tmp = rospy.get_param("/maintenance_charger")
                rospy.loginfo("[" + self.node_name + "]" +" self.charger = "+str(self.charger)+ " rosparam /maintenance_charger = "+str(tmp))
                self.tl_freq_before = self.tl_freq
            '''

            new_turn_type = self.getTurnType(self.charger, tag_id, self.maintenance_state)
            if new_turn_type != -1:
                rospy.loginfo("["+self.node_name+"]"+"State: " + str(self.maintenance_state) + ", Charger: " + str(self.charger) + ", tag_ID: "  + str(tag_id) + " - therefore driving " + str(new_turn_type))
                turn_type = new_turn_type
            else:
                if str(tag_id) in list(self.maintenance_exit.keys()) : 
                    rospy.loginfo("["+self.node_name+"]"+"I am at Maintenance Exit")
                elif self.maintenance_state != "WAY_TO_MAINTENANCE":
                    rospy.loginfo("["+self.node_name+"]"+"Active navigation running, but unknown tag ID " + str(tag_id))

        msg.turn_type = turn_type
        self.pub_turn_type.publish(msg)


    ##### END callback functions #####

    ##### BEGIN internal functions #####

    # Check if tag_id&turn_type are in dictionary
    def isInDict(self, tag_id, turn_type, dictionary):
        inDict = False
        if str(tag_id) in dictionary:
            turns = dictionary[str(tag_id)]
            if isinstance(turns, int): # single turntypes for this tag
                inDict = True if (turn_type == turns) else False
            else: # multiple turntypes for this tag
                inDict = True if (turn_type in turns) else False
        return inDict

    # Function to change MT state
    def changeMTState(self, state):
        # Change internal state and publish information
        self.maintenance_state = state
        maintenance_msg = MaintenanceState()
        maintenance_msg.state = state
        self.pub_maintenance_state.publish(maintenance_msg)

        rospy.loginfo("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
        rospy.loginfo("["+self.node_name+"]"+" MT State: " + str(state))
        rospy.loginfo("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")

        self.active_navigation = self.maintenance_state in ["WAY_TO_CHARGING", "WAY_TO_CALIBRATING", "WAY_TO_CITY", "WAY_TO_MAINTENANCE"]


    # Returns the turn type for an intersection to get to charger
    def getTurnType(self, chargerID, tagID, maintenance_state):
        if maintenance_state == "WAY_TO_CHARGING":
            path = (self.stations['station' + str(chargerID)])['path_in']
        if maintenance_state == "WAY_TO_CALIBRATING":
            path = (self.stations['station' + str(chargerID)])['path_calib']
        if maintenance_state == "WAY_TO_CITY":
            path = self.path_to_city
        if maintenance_state == "WAY_TO_MAINTENANCE":
            #TODO:after demo change this part to self.maintenance_entrance
            path = self.maintenance_path

        turn = -1
        # Get turn type if defined in YAML file
        turn = path[str(tagID)] if str(tagID) in path else turn

        return turn

    ##### END internal functions #####

    ##### BEGIN standard functions #####
    def setupParams(self):
        self.maintenance_entrance = self.setupParam("~maintenance_entrance", 0)
        self.maintenance_path = self.setupParam("~maintenance_path",0)
        self.maintenance_exit = self.setupParam("~maintenance_exit", 0)
        self.stations = self.setupParam("~charging_stations", 0)
        self.calibration_station = self.setupParam("~calibration_station", 0)
        self.path_to_city = self.setupParam("~path_to_city", 0)
        if not rospy.has_param("/maintenance_charger"): 
            self.charger = self.setupParam("/maintenance_charger", 1)
        self.MT_TAG = self.setupParam("/maintenance_intersection", 0)
        self.maintenance_intersection = self.setupParam("~maintenance_intersection",0)
        self.charging_stop_signs = self.setupParam("~charging_stations/stop_signs", 0)
        self.protocol = self.setupParam("~LED_protocol",[])



    def updateParams(self,event):
        self.maintenance_entrance = rospy.get_param("~maintenance_entrance")
        self.maintenance_path = rospy.get_param("~maintenance_path")
        self.maintenance_exit = rospy.get_param("~maintenance_exit")
        self.stations = rospy.get_param("~charging_stations")
        self.calibration_station = rospy.get_param("~calibration_station")
        self.path_to_city = rospy.get_param("~path_to_city")
        self.charger = rospy.get_param("/maintenance_charger")
        self.maintenance_intersection = rospy.get_param("~maintenance_intersection")
        rospy.set_param("/maintenance_intersection", self.maintenance_intersection)


        self.charging_stop_signs = rospy.get_param("~charging_stations/stop_signs")
        self.protocol = rospy.get_param("~LED_protocol")

        self.charger_frequencies = [self.protocol['signals']['CAR_SIGNAL_A_OLD']['frequency'], self.protocol['signals']['CAR_SIGNAL_A']['frequency'], self.protocol['signals']['CAR_SIGNAL_GREEN']['frequency'], self.protocol['signals']['traffic_light_go']['frequency']]
        #rospy.loginfo("["+self.node_name+"]TL_detect: "+str(self.TL_detect)+" det_completed: "+str(self.detection_completed))

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def onShutdown(self):
        rospy.loginfo("["+self.node_name+"]"+" Shutdown.")

    ##### END standard functions #####

if __name__ == '__main__':
    rospy.init_node('maintenance_control_node',anonymous=False)
    maintenance_control_node = MaintenanceControlNode()
    rospy.on_shutdown(maintenance_control_node.onShutdown)
    rospy.spin()
