import time
import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range

class Ultrasonic(Node):
  def __init__(self):
    super().__init__('ultrasonic')
    self.publisher_ = self.create_publisher(Range, 'ultrasonic', 10)
    timer_period = 1  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    
    GPIO.setwarnings(False)
    self.trigger_pin = 27
    self.echo_pin = 22
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(self.trigger_pin,GPIO.OUT)
    GPIO.setup(self.echo_pin,GPIO.IN)
    
  def timer_callback(self):
    msg = Range()
    msg.radiation_type = 0
    msg.range = float(self.get_distance())
    #self.get_logger().info("sent " + str(msg.range))
    self.publisher_.publish(msg)

  def send_trigger_pulse(self):
    GPIO.output(self.trigger_pin,True)
    time.sleep(0.00015)
    GPIO.output(self.trigger_pin,False)

  def wait_for_echo(self,value,timeout):
    count = timeout
    while GPIO.input(self.echo_pin) != value and count>0:
      count = count-1
    
  def get_distance(self):
    distance_cm = 0
    while distance_cm < 1 or distance_cm > 1500:
      self.send_trigger_pulse()
      self.wait_for_echo(True,10000)
      start = time.time()
      self.wait_for_echo(False,10000)
      finish = time.time()
      pulse_len = finish-start
      distance_cm = pulse_len/0.000058
    return float(distance_cm)
  

def main(args=None):
  rclpy.init(args=args)

  ultrasonic = Ultrasonic()

  rclpy.spin(ultrasonic)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  ultrasonic.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
