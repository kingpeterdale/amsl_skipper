import rclpy
from rclpy.node import Node

from aruco_markers_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .utils import *


class ArucoLocalizer(Node):
  def __init__(self):
    super().__init__('aruco_localizer')
    self.markers_sub = self.create_subscription(MarkerArray, 'aruco/markers',self.markers_cb, 10)
    self.tf_broadcaster = TransformBroadcaster(self)
    self.tf_static_broadcaster = StaticTransformBroadcaster(self)
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    self.x = 0.
    self.y = 0.
    self.z = 0.

    # Broadcast static transform from camera to base_link

    t = TransformStamped()

    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'base_link'
    t.child_frame_id = 'camera'

    t.transform.translation.x = 0.
    t.transform.translation.y = 0.
    t.transform.translation.z = 0.
    quat = quat_from_euler(-1.5708,0.,0.)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    self.tf_static_broadcaster.sendTransform(t)
    

  def markers_cb(self, msg):
    for marker in msg.markers:
      if marker.id == 0:
        pass
        # Using Andrej Babinex et al. / Procedia Engineering 96 (2014)
        '''pos = marker.pose.pose.position
        rot = marker.pose.pose.orientation
        theta = math.atan2(2*(rot.x*rot.y - rot.w*rot.z), rot.w**2 - rot.x**2 - rot.y**2 + rot.z**2)
        xw = pos.x*math.cos(theta) - pos.z*math.sin(theta)
        yw = pos.z*math.cos(theta) - pos.x*math.sin(theta)
        print(f'{xw:.1f},{yw:.1f},{180. + math.degrees(theta):.0f}')'''

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
        #print(f'{self.x:.1f} {self.y:.1f} {self.z:.1f}')

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
