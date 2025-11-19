import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

from threading import Thread

import serial
from datetime import datetime
import numpy as np

class SikInterface(Node):
  def __init__(self):
    # ROS2 Node configure
    super().__init__('sik_interface')
    self.subscription = self.create_subscription(LaserScan,'scan',self.laser_cb, 10)
    self.timer = self.create_timer(0.3,self.timer_cb)
 
    # SIK Radio Interface
    try:
      self.sik_port = serial.Serial('/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_D30GL9AC-if00-port0',115200,timeout=10)
      self.get_logger().info('Serial Port opened for SIK Radio')
      self.sik_read_thread = Thread(target=self.sik_read,daemon=True)
      self.sik_read_thread.start()
    except:
      self.get_logger().error('ERROR OPENING SIK SERIAL PORT')

    try:
      self.llc_port = serial.Serial('/dev/serial/by-id/usb-Arduino_Nano_ESP32_3C8427C568B0-if01',9600,timeout=10)
      self.get_logger().info('Serial Port Opening for LLC')
      self.llc_read_thread = Thread(target=self.llc_read, daemon=True)
      self.llc_read_thread.start()
    except:
      self.get_logger().error('ERROR OPENING LLC PORT')


    # laser data
    self.laser_scan = ""
    self.laser_dt = datetime.now()
    self.laser_logfile = open(f'laser_{datetime.now():%Y%m%d_%H%M%S}.log','w')


  def laser_cb(self,msg):
    ''' Callback for new LaserScan msg
          Normalises angles from -180 to 180
          reduces angular range to -60 to 60
          converts range to UNIN8 with 10cm resolution
    '''
    angles = np.degrees(np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges)))
    #angles = np.mod((angles + 180),360) - 180
    #i = np.sort(np.where((angles>120)|(angles<-120)))
    #print(angles[0],angles[-1],len(angles))
    ranges = np.array(msg.ranges)#[i]
    self.laser_scan = np.where((ranges<17)&(ranges>1), ranges*10, 0).astype(np.uint8)
    self.laser_dt = datetime.now()
    return
    try:
      nw = self.sik_port.write(f'{self.laser_dt:%H:%M:%S.%f}'.encode() + self.laser_scan.tobytes() + b'\xFF')
    except:
      pass
    #print(self.laser_scan)
    
    #self.laser_scan = np.array([int(r*10) if (r<17 and r>1) else 0 for r in np.array(msg.ranges)[i]],dtype=np.uint8)
    #self.laser_logfile.write(f'[{datetime.now():%H:%M:%S.%f}]' + ','.join([f'{r:.1f}' for r in msg.ranges]))
    #self.laser_logfile.write('\n')


  def timer_cb(self):
    try:
      nw = self.sik_port.write(f'{self.laser_dt:%H:%M:%S.%f}'.encode() + self.laser_scan.tobytes() + b'\xFF')
    except:
      pass
    #print(nw)

  def sik_read(self):
    ''' continuously reads the serial port attached to the SIK radio
          this function should be run as a thread
    '''
    self.get_logger().info('SIK Radio Read Thread Started')
    while True:
        msg = self.sik_port.readline()
        if len(msg) > 0:
          print(msg)
          try:
            self.llc_port.write(msg)
          except:
            print('could not write to LLC')


  def llc_read(self):
    self.get_logger().info('LLC Read Thread Started')
    while True:
      msg = self.llc_port.readline()
      if len(msg) > 5:
        self.laser_logfile.write(f'[{datetime.now():%H:%M:%S.%f}]' + msg.decode().strip())
        self.laser_logfile.write('\n')
        self.laser_logfile.flush()
        print(msg.decode().strip())


def main(args=None):
  rclpy.init(args=args)
  sik_interface = SikInterface()

  try:
    rclpy.spin(sik_interface)
  except KeyboardInterrupt:
    exit()

  sik_interface.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
