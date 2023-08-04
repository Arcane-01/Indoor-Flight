#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

global offboard_detect # Flag to detect if OFFBOARD mode has been enabled and the drone has flied
global arm_detect # Flag to check if the drone has been armed once

# Both the flags are set to False initially
offboard_detect = False
arm_detect = False

def state_cb(msg):
    global current_state
    current_state = msg


if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    # Subscriber for state
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

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

    # The desired setpoints 
    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 1.5

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
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
            # The drone is armed if it is disarmed, the current mode is not Land and the flag arm_detect is not set 
            if(not current_state.armed and current_state.mode != "AUTO.LAND" and (not arm_detect) and (rospy.Time.now() - last_req) > rospy.Duration(10.0)):
                if(arming_client.call(arm_cmd).success == True):
                    arm_detect = True
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        # Print LAND enabled if the current mode is Land
        if(current_state.mode == "AUTO.LAND" and (rospy.Time.now() - last_req) > rospy.Duration(10.0)):
            rospy.loginfo("LAND enabled")
            last_req = rospy.Time.now()

        local_pos_pub.publish(pose)

        rate.sleep()
