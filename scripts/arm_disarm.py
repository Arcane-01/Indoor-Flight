#!/usr/bin/python3

import rospy
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.msg import State

current_state = State()

global arm_detect # Flag to check if the drone has been armed once
arm_detect = False 

# callback functions to update the current state of the drone

def state_cb(msg): # subscribed to the mavros/state topic
    global current_state
    current_state = msg

if __name__ == "__main__":
    rospy.init_node("arm_disarm_py")

    # Subscribers for state, position and velocity
    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb) 

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rate = rospy.Rate(10)  # Rate of 10 Hz
    rospy.loginfo("Code in the main loop")
    # Wait for Flight Controller connection
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    disarm_cmd = CommandBoolRequest()
    disarm_cmd.value = False

    arming_duration = rospy.Duration(5.0)  # Duration to keep the drone armed

    last_req = rospy.Time.now()

    while not rospy.is_shutdown():

        # If the drone is not armed and the flag is not set, the drone is armed
        if not current_state.armed and not arm_detect and (rospy.Time.now() - last_req) > rospy.Duration(1.0):
            if arming_client.call(arm_cmd).success:
                rospy.loginfo("Vehicle armed")
                arm_detect = True  # Set the armed flag to True once the drone is armed
            last_req = rospy.Time.now()

        # Disarm the vehicle after the specified duration
        if current_state.armed and (rospy.Time.now() - last_req) > arming_duration:
            if arming_client.call(disarm_cmd).success:
                rospy.loginfo("Vehicle disarmed")
                # armed = False  # Set the armed flag to False
            last_req = rospy.Time.now()

        rate.sleep()
