import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped

class Pose(object):
    def __init__(self):
        self._pose = TransformStamped()
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer)

    def get_pose(self):
        try:
            pose = self._buffer.lookup_transform('world', 
                                                 'glove', 
                                                 rospy.Time(0))
            self._pose = pose
        except tf2_ros.TransformException:
            pass
        return self._pose
