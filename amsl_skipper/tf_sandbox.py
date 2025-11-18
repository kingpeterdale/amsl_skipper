import rclpy
from rclpy.node import Node

from aruco_markers_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster

from .utils import *


class ArucoLocalizer(Node):
  def __init__(self):
    super().__init__('aruco_localizer')
    self.markers_sub = self.create_subscription(MarkerArray, 'aruco/markers',self.markers_cb, 10)
    self.tf_broadcaster = TransformBroadcaster(self)

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    self.x = 0.
    self.y = 0.
    self.z = 0.

  def markers_cb(self, msg):
    for marker in msg.markers:
      if marker.id == 0:
        pos = marker.pose.pose.position
        self.x = -0.5*pos.x + 0.5*self.x
        self.y = -0.5*pos.z + 0.5*self.y
        self.z = -0.5*pos.y + 0.5*self.z
        '''# get inverse transform to localise Robot
        try:
          t = self.tf_buffer.lookup_transform(
            'camera','aruco_marker_0',  rclpy.time.Time())
        except TransformException as ex:
          self.get_logger().info(f'Could not transform')
          return
        #t.header.frame_id = 'map'
        #t.child_frame_id = 'base_link'
        #self.tf_broadcaster.sendTransform(t)
        #print('Sent TF map to base_link')
        r,p,y = euler_from_quat(t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w)        
        print(f'{r:.1f},{p:.1f},{y:.1f}')
        '''
        print(f'{self.x:.1f} {self.y:.1f} {self.z:.1f}')

def main(args=None):
  rclpy.init(args=args)
  aruco_localizer = ArucoLocalizer()
  try:
    rclpy.spin(aruco_localizer)
  except KeyboardInterrupt:
    exit()
  
  aruco_localizer.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
