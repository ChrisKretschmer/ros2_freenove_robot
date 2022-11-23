import time
import smbus
import rclpy

from robot.PCA9685 import PCA9685
from rclpy.node import Node

from geometry_msgs.msg import Twist

class Motor(Node):
  def __init__(self):
    super().__init__('motor')
    self.pwm = PCA9685(0x40, debug=True)
    self.pwm.setPWMFreq(50)
    self.subscription = self.create_subscription(
      Twist,
      'teleop',
      self.listener_callback,
      10)
    self.subscription
    
  def listener_callback(self, msg):
    if(msg.linear.x != 0):
      self.setMotorModel(int(2000 * msg.linear.x), int(2000 * msg.linear.x) ,int(2000 * msg.linear.x), int(2000 * msg.linear.x))
    elif(msg.angular.z > 0):
      self.setMotorModel(-500,-500,2000,2000)
    elif(msg.angular.z < 0):
      self.setMotorModel(2000,2000,-500,-500)
  
  def duty_range(self,duty1,duty2,duty3,duty4):
    if duty1>4095:
      duty1=4095
    elif duty1<-4095:
      duty1=-4095        
    
    if duty2>4095:
      duty2=4095
    elif duty2<-4095:
      duty2=-4095
        
    if duty3>4095:
      duty3=4095
    elif duty3<-4095:
      duty3=-4095
        
    if duty4>4095:
      duty4=4095
    elif duty4<-4095:
      duty4=-4095
    return duty1,duty2,duty3,duty4
  
  def left_Upper_Wheel(self,duty):
    if duty>0:
      self.pwm.setMotorPwm(0,0)
      self.pwm.setMotorPwm(1,duty)
    elif duty<0:
      self.pwm.setMotorPwm(1,0)
      self.pwm.setMotorPwm(0,abs(duty))
    else:
      self.pwm.setMotorPwm(0,4095)
      self.pwm.setMotorPwm(1,4095)
  def left_Lower_Wheel(self,duty):
    if duty>0:
      self.pwm.setMotorPwm(3,0)
      self.pwm.setMotorPwm(2,duty)
    elif duty<0:
      self.pwm.setMotorPwm(2,0)
      self.pwm.setMotorPwm(3,abs(duty))
    else:
      self.pwm.setMotorPwm(2,4095)
      self.pwm.setMotorPwm(3,4095)
  def right_Upper_Wheel(self,duty):
    if duty>0:
      self.pwm.setMotorPwm(6,0)
      self.pwm.setMotorPwm(7,duty)
    elif duty<0:
      self.pwm.setMotorPwm(7,0)
      self.pwm.setMotorPwm(6,abs(duty))
    else:
      self.pwm.setMotorPwm(6,4095)
      self.pwm.setMotorPwm(7,4095)
  def right_Lower_Wheel(self,duty):
    if duty>0:
      self.pwm.setMotorPwm(4,0)
      self.pwm.setMotorPwm(5,duty)
    elif duty<0:
      self.pwm.setMotorPwm(5,0)
      self.pwm.setMotorPwm(4,abs(duty))
    else:
      self.pwm.setMotorPwm(4,4095)
      self.pwm.setMotorPwm(5,4095)
  def setMotorModel(self,duty1,duty2,duty3,duty4):
    duty1,duty2,duty3,duty4=self.duty_range(duty1,duty2,duty3,duty4)
    self.left_Upper_Wheel(duty1)
    self.left_Lower_Wheel(duty2)
    self.right_Upper_Wheel(duty3)
    self.right_Lower_Wheel(duty4)
 
def main(args=None):
  rclpy.init(args=args)

  motor = Motor()
  motor.setMotorModel(0,0,0,0)

  rclpy.spin(motor)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  motor.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
