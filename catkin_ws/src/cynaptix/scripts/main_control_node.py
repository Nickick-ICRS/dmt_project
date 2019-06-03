#!/usr/bin/env python

import rospy

from main_control.frame import Frame
from main_control.glove import Glove
from main_control.pose import Pose

if __name__ == '__main__':
    rospy.init_node("main_controller")

    frame = Frame()
    glove = Glove()
    pose = Pose()

    # Read torque thresholds from parameter server
    frame_torque_thresh = [0, 0, 0, 0, 0]
    max_frame_torque = [0, 0, 0, 0, 0]
    max_tactile_response = 70

    while not rospy.is_shutdown():
        # Get up to date measurements
        glove_pos = pose.get_pose()
        glove_servos = glove.get_servo_pos()
        frame_pos = frame.get_motor_pos()
        frame_torque = frame.get_motor_torques()

        # Store the tactile response per axis
        tactile_resp = [0, 0, 0, 0, 0]

        # Extract yaw from quaternion
        glove_yaw = tf_conversions.transformations.euler_from_quaternion(
                        glove_pos.orientation)[2]

        # Calculate the grabbing angle
        glove_grab = (glove_servos[0] + glove_servos[1] + glove_servos[3])/3

        # Calculate x, y, z, yaw and grabber errors
        err = [glove_pos.transform.translation.x - frame_pos[0],
               glove_pos.transform.translation.y - frame_pos[1],
               glove_pos.transform.translation.z - frame_pos[2],
               frame_torque - frame_pos[3],
               glove_grab - frame_pos[4]]

        # If the error is below a certain threshold then we're good enough
        if abs(err[0]) < err_thresh[0]:
            pass
        # If the torque is below the threshold then move the frame
        elif frame_torque[0] < frame_torque_thresh[0]:
            frame_pos[0] = glove_pos.transform.translation.x
        # If the torque is higher than the threshold the glove can't move
        # However, we can output tactile feedback
        else:
            tactile_resp[0] = (frame_torque[0] - frame_torque_thresh[0]) \
                            * max_tactile_response / max_frame_torque[0];
            # Tactile should be a multiple of 10 (helps eliminate noise)
            tactile_resp[0] = int(tactile_resp[0] / 10.0 + 0.5) * 10
