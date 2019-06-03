import threading
import rospy

from cynaptix.msg import FrameTarget
from cynaptix.msg import FrameMeasuredData

class Frame(object):
    def __init__(self):
        self._motor_torques = [0, 0, 0, 0, 0]
        self._motor_pos = [0, 0, 0, 0, 0]
        self._target_motor_pos = [0, 0, 0, 0, 0]
        self._sub = rospy.Subscriber('frame_measured_data',
                                     FrameMeasuredData,
                                     self.data_measured_callback)
        self._pub = rospy.Publisher('frame_target',
                                    FrameTarget,
                                    queue_size=1)
        self._lock = threading.Lock()

    def data_measured_callback(self, data):
        # Prevent race conditions
        self._lock.acquire()
        self._motor_pos = [data.x_pos,
                           data.y_pos,
                           data.z_pos,
                           data.theta_pos,
                           data.grabber_pos]
        self._motor_torques = [data.x_torque,
                               data.y_torque,
                               data.z_torque,
                               data.theta_torque,
                               data.grabber_torque]
        self._lock.release()

        # Send current targets back
        self.publish_data()

    def publish_data(self):
        msg = FrameTarget()
        # Prevent race conditions
        self._lock.acquire()
        msg.x_pos = self._target_motor_pos[0]
        msg.y_pos = self._target_motor_pos[1]
        msg.z_pos = self._target_motor_pos[2]
        msg.theta_pos = self._target_motor_pos[3]
        msg.grabber_pos = self._target_motor_pos[4]
        self._lock.release()

    def get_motor_pos(self):
        # Prevent race conditions
        self._lock.acquire()
        pos = self._motor_pos
        self._lock.release()
        return pos

    def get_motor_torques(self):
        # Prevent race conditions
        self._lock.acquire()
        torques = self._motor_torques
        self._lock.release()
        return torques

    def set_motor_target(self, targets):
        # Prevent race conditions
        self._lock.acquire()
        self._target_motor_pos = targets
        self._lock.release()

    def set_vibration_values(self, vibrations):
        # Prevent race conditions
        self._lock.acquire()
        self._vibrations = vibrations
        self._lock.release()
