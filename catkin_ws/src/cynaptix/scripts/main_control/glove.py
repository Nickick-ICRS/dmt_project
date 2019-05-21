import threading
import rospy

from cynaptix.msg import GloveTarget
from cynaptix.msg import GloveMeasuredData

class Glove(object):
    def __init__(self):
        self._vibrations = [0, 0, 0,
                            0, 0, 0,
                            0, 0, 0]
        self._servo_pos = [0, 0, 0]
        self._target_servo_pos = [0, 0, 0]
        self._sub = rospy.Subscriber('glove_measured_data',
                                     GloveMeasuredData,
                                     self.data_measured_callback)
        self._pub = rospy.Publisher('glove_target',
                                    GloveTarget,
                                    queue_size=1)
        self._lock = threading.Lock()

    def data_measured_callback(self, data):
        # Prevent race conditions
        self._lock.acquire()
        self._servo_pos = [data.thumb_pos,
                           data.index_pos,
                           data.middle_pos]
        self._lock.release()
        # Send current targets back
        self.publish_data()

    def publish_data(self):
        msg = GloveTarget()
        # Prevent race conditions
        self._lock.acquire()
        msg.thumb_target = self._target_servo_pos[0]
        msg.index_target = self._target_servo_pos[1]
        msg.middle_target = self._target_servo_pos[2]
        self._lock.release()

    def get_servo_pos(self):
        # Prevent race conditions
        self._lock.acquire()
        pos = self._servo_pos
        self._lock.release()
        return pos

    def set_servo_target(self, targets):
        # Prevent race conditions
        self._lock.acquire()
        self._target_servo_pos = targets
        self._lock.release()

    def set_vibration_values(self, vibrations):
        # Prevent race conditions
        self._lock.acquire()
        self._vibrations = vibrations
        self._lock.release()
