#!/usr/bin/python3

# Code to make the UAV follow a set of waypoints with the low-level controller of PX4

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()
current_pose = PoseStamped()

global offboard_detect # Flag to detect if OFFBOARD mode has been enabled and the drone has flied
global arm_detect # Flag to check if the drone has been armed once

# Both the flags are set to False initially
offboard_detect = False
arm_detect = False

# array of tolerance value used for switching to the next waypoint
position_tolerance_arr = [0.2,0.17]

# List of waypoints for the drone to follow
waypoints = [
    [0, 0, 1.5],    # Waypoint 1: x=0, y=0, z=1.5
    [1.0, 1.0, 1.5],
    [1.0, -1.0, 1.5],    
    [-1.0, -1.0, 1.5],   
    [-1.0, 1.0, 1.5] 
]

current_waypoint_index = 0 # the waypoint index is initially set to zero

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg): # subscribed to the mavros/local_position/pose topic
    global current_pose
    current_pose = msg

if __name__ == "__main__":
    rospy.init_node("position_py")

    
    # Subscribers for state, position and velocity
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb, queue_size=1)
    pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=pose_cb, queue_size=1) # used for finding the current roll of the quadrotor 

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    # initail pose inputs for enabling offboard mode
    pose_i = PoseStamped()

    pose_i.pose.position.x = 0
    pose_i.pose.position.y = 0
    pose_i.pose.position.z = 0

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose_i)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):

        # OFFBOARD mode is set if the current mode is neither Offboard nor Land and the flag offboard_detect is not set
        if(current_state.mode != "OFFBOARD" and current_state.mode != "AUTO.LAND" and (not offboard_detect) and (rospy.Time.now() - last_req) > rospy.Duration(10.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                offboard_detect = True
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()

        else:
            #  The drone is armed if it is disarmed, the current mode is not Land and the flag arm_detect is not set 
            if(not current_state.armed and current_state.mode != "AUTO.LAND" and (not arm_detect) and (rospy.Time.now() - last_req) > rospy.Duration(10.0)):
                if(arming_client.call(arm_cmd).success == True):
                    arm_detect = True
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        # Print LAND enabled if the current mode is Land
        if(current_state.mode == "AUTO.LAND" and (rospy.Time.now() - last_req) > rospy.Duration(10.0)):
            rospy.loginfo("LAND enabled")
            last_req = rospy.Time.now()

        # Current position of the Quadcopter used for switching to the next waypoint
        curr_pos = current_pose.pose.position

        # the current waypoint obtained from the waypoints array
        current_waypoint = waypoints[current_waypoint_index]

        # the tolerance value is selected based on the value of the current index
        if current_waypoint_index == 0:
            position_tolerance = position_tolerance_arr[0]
        else:
            position_tolerance = position_tolerance_arr[1]

        # Switch the waypoint index if the absoulte error value b/w current position and current waypoint for x, y and z is less than the tolerance value
        if (abs(curr_pos.x - current_waypoint[0]) < position_tolerance and abs(curr_pos.y - current_waypoint[1]) < position_tolerance and abs(curr_pos.z - current_waypoint[2]) < position_tolerance):
            
            # Increment waypoint index
            current_waypoint_index += 1
            
            # If the wapoint value exceeds the length of waypoints array, set it to the last waypoint
            if current_waypoint_index >= len(waypoints):
                current_waypoint_index = len(waypoints)-1
                
            # Print the current waypoint index if it's not the final waypoint
            if current_waypoint_index < (len(waypoints)-1):
                rospy.loginfo("Switching to waypoint index: {}".format(current_waypoint_index))

        target_waypoint = waypoints[current_waypoint_index]
        
        # the target Pose for x, y, z position 
        target_pose = PoseStamped()

        target_pose.pose.position.x = target_waypoint[0]
        target_pose.pose.position.y = target_waypoint[1]
        target_pose.pose.position.z = target_waypoint[2]
        
        local_pos_pub.publish(target_pose)

        rate.sleep()