#!/usr/bin/env python

import rospy
from math import sin, cos, asin

from main_control.frame import Frame
from main_control.glove import Glove
from main_control.pose import Pose

def calculate_tactile_per_motor(tactile_axis, errors, yaw):
    # X and Y depend on the yaw
    # Yaw = 0 corresponds to motion along the y axis
    # 0.4 * (X and Y) plus 0.2 * grabber plus 0.4 * (positive)yaw
    xy = (sin(yaw + 0.7853982) * tactile_axis[0] \
         + cos(yaw + 0.7853982) * tactile_axis[1]) 
    tmb_l = 0.4 * (xy if xy > 0 else 0) + 0.2 * tactile_axis[4] \
            + 0.4*(1 if errors[3] > 0 else 0) * tactile_axis[3]
    # 0.7 * Z plus 0.3 * grabber
    tmb_t = 0.7 * tactile_axis[2] + 0.3 * tactile_axis[4]
    # 0.4 * (X and Y) plus 0.2 * grabber plus 0.4 * (negative)yaw
    xy = (sin(yaw - 0.7853982) * tactile_axis[0] \
         + cos(yaw - 0.7853982) * tactile_axis[1]) 
    tmb_r = 0.4 * (xy if xy > 0 else 0) + 0.2 * tactile_axis[4] \
            + 0.4*(1 if errors[3] < 0 else 0) * tactile_axis[3]

    xy = (sin(yaw - 0.7853982) * tactile_axis[0] \
         - cos(yaw - 0.7853982) * tactile_axis[1]) 
    indx_l = 0.4 * (xy if xy > 0 else 0) + 0.2 * tactile_axis[4] \
            + 0.4*(1 if errors[3] < 0 else 0) * tactile_axis[3]
    indx_t = 0.7 * tactile_axis[2] + 0.3 * tactile_axis[4]
    xy = (sin(yaw + 0.7853982) * tactile_axis[0] \
         - cos(yaw + 0.7853982) * tactile_axis[1]) 
    indx_r = 0.4 * (xy if xy > 0 else 0) + 0.2 * tactile_axis[4] \
            + 0.4*(1 if errors[3] > 0 else 0) * tactile_axis[3]

    xy = (sin(yaw - 0.7853982) * tactile_axis[0] \
         - cos(yaw - 0.7853982) * tactile_axis[1]) 
    mid_l = 0.4 * (xy if xy > 0 else 0) + 0.2 * tactile_axis[4] \
            + 0.4*(1 if errors[3] < 0 else 0) * tactile_axis[3]
    mid_t = 0.7 * tactile_axis[2] + 0.3 * tactile_axis[4]
    xy = (sin(yaw + 0.7853982) * tactile_axis[0] \
         - cos(yaw + 0.7853982) * tactile_axis[1]) 
    mid_r = 0.4 * (xy if xy > 0 else 0) + 0.2 * tactile_axis[4] \
            + 0.4*(1 if errors[3] > 0 else 0) * tactile_axis[3]

    return [tmb_l , tmb_t , tmb_r , \
            indx_l, indx_t, indx_r, \
            mid_l , mid_t , mid_r]

if __name__ == '__main__':
    rospy.init_node("main_controller")

    frame = Frame()
    glove = Glove()
    pose = Pose()

    # Read torque thresholds from parameter server
    frame_torque_thresh = [10000, 10000, 10000, 10000, 10000]
    max_frame_torque = [10000, 10000, 10000, 10000, 10000]
    err_thresh = [0, 0, 0, 0, 0]
    max_tactile_response = 70

    while not rospy.is_shutdown():
        servo_targets = [0, 0, 0]
        # Get up to date measurements
        glove_pos = pose.get_pose()
        glove_servos = glove.get_servo_pos()
        frame_pos = frame.get_motor_pos()
        # Decrease frame yaw by 90 and convert to radians
        frame_pos[3] = (frame_pos[3] / 180 - 0.5) * 3.1415926
        frame_torque = frame.get_motor_torques()

        # Store the tactile response per axis
        tactile_resp = [0, 0, 0, 0, 0]

        # Extract yaw from quaternion
        glove_yaw = asin(glove_pos.transform.rotation.z) * 2

        # Calculate the grabbing angle
        glove_grab = (glove_servos[0] + glove_servos[1] + glove_servos[2])/3

        # Calculate x, y, z, yaw and grabber errors
        err = [glove_pos.transform.translation.x - frame_pos[0],
               glove_pos.transform.translation.y - frame_pos[1],
               glove_pos.transform.translation.z - frame_pos[2],
               glove_yaw - frame_pos[3],
               glove_grab - frame_pos[4]]

        ### X AXIS ###

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

        ### Y AXIS ###

        # If the error is below a certain threshold then we're good enough
        if abs(err[1]) < err_thresh[1]:
            pass
        # If the torque is below the threshold then move the frame
        elif frame_torque[1] < frame_torque_thresh[1]:
            frame_pos[1] = glove_pos.transform.translation.y
        # If the torque is higher than the threshold the glove can't move
        # However, we can output tactile feedback
        else:
            tactile_resp[1] = (frame_torque[1] - frame_torque_thresh[1]) \
                            * max_tactile_response / max_frame_torque[1];
            # Tactile should be a multiple of 10 (helps eliminate noise)
            tactile_resp[1] = int(tactile_resp[1] / 10.0 + 0.5) * 10

        ### Z AXIS ###

        # If the error is below a certain threshold then we're good enough
        if abs(err[2]) < err_thresh[2]:
            pass
        # If the torque is below the threshold then move the frame
        elif frame_torque[2] < frame_torque_thresh[2]:
            frame_pos[2] = glove_pos.transform.translation.z
        # If the torque is higher than the threshold the glove can't move
        # However, we can output tactile feedback
        else:
            tactile_resp[2] = (frame_torque[2] - frame_torque_thresh[2]) \
                            * max_tactile_response / max_frame_torque[2];
            # Tactile should be a multiple of 10 (helps eliminate noise)
            tactile_resp[2] = int(tactile_resp[2] / 10.0 + 0.5) * 10

        ### YAW AXIS ###

        # If the error is below a certain threshold then we're good enough
        if abs(err[3]) < err_thresh[3]:
            pass
        # If the torque is below the threshold then move the frame
        elif frame_torque[3] < frame_torque_thresh[3]:
            # Convert to frame co-ordinate frame and degrees
            frame_pos[3] = glove_yaw * 180 / 3.1415925 # - 180
        # If the torque is higher than the threshold the glove can't move
        # However, we can output tactile feedback
        else:
            tactile_resp[3] = (frame_torque[3] - frame_torque_thresh[3]) \
                            * max_tactile_response / max_frame_torque[3];
            # Tactile should be a multiple of 10 (helps eliminate noise)
            tactile_resp[3] = int(tactile_resp[3] / 10.0 + 0.5) * 10

        ### GRABBER AXIS ###

        # If the error is below a certain threshold then we're good enough
        if abs(err[4]) < err_thresh[4]:
            pass
        # If the torque is below the threshold then move the frame
        elif frame_torque[4] < frame_torque_thresh[4]:
            frame_pos[4] = glove_grab;
        # If the torque is higher than the threshold the glove can't move
        # However, we can output tactile feedback
        else:
            tactile_resp[4] = (frame_torque[4] - frame_torque_thresh[4]) \
                            * max_tactile_response / max_frame_torque[4];
            # Tactile should be a multiple of 10 (helps eliminate noise)
            tactile_resp[4] = int(tactile_resp[4] / 10.0 + 0.5) * 10

        full_tactile_response = calculate_tactile_per_motor(tactile_resp, 
                                                            err, 
                                                            frame_pos[3])

        frame.set_motor_target(frame_pos)
        glove.set_servo_target(servo_targets)
        glove.set_vibration_values(full_tactile_response)
