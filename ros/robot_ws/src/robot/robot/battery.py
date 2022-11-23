import time
import smbus
import rclpy
from rclpy.node import Node


from sensor_msgs.msg import BatteryState

class Battery(Node):
  def __init__(self):
    super().__init__('battery')
    self.bus = smbus.SMBus(1)
    # I2C address of the device
    self.ADDRESS = 0x48
    
    # PCF8591 Command
    self.PCF8591_CMD =0x40  #Command
    
    # ADS7830 Command 
    self.ADS7830_CMD = 0x84 # Single-Ended Inputs
    
    for i in range(3):
      aa=self.bus.read_byte_data(self.ADDRESS,0xf4)
      if aa < 150:
        self.Index="PCF8591"
      else:
        self.Index="ADS7830"
        
    self.get_logger().info("found ADC " + self.Index)
    self.publisher_ = self.create_publisher(BatteryState, 'battery', 10)
    timer_period = 1  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    
  def timer_callback(self):
    msg = BatteryState()
    msg.power_supply_technology = 3
    msg.power_supply_status = 2
    msg.power_supply_health = 0
    
    msg.voltage = float(self.get_voltage())
    if(msg.voltage > 3):
      msg.present = True
    else:
      msg.present = False
    
    self.get_logger().info("sent " + str(msg.voltage))
    self.publisher_.publish(msg)
    
  def get_voltage(self):
    if self.Index=="PCF8591":
      data=self.recvPCF8591(2) * 3
    elif self.Index=="ADS7830":
      data=self.recvADS7830(2) * 3
    return data
  
  def recvPCF8591(self,channel):#PCF8591 write DAC value
    while(1):
      value1 = self.analogReadPCF8591(channel)   #read the ADC value of channel 0,1,2,
      value2 = self.analogReadPCF8591(channel)
      if value1==value2:
        break;
    voltage = value1 / 256.0 * 3.3  #calculate the voltage value
    voltage = round(voltage,2)
    return voltage
  
  def recvADS7830(self,channel):
    """Select the Command data from the given provided value above"""
    COMMAND_SET = self.ADS7830_CMD | ((((channel<<2)|(channel>>1))&0x07)<<4)
    self.bus.write_byte(self.ADDRESS,COMMAND_SET)
    while(1):
      value1 = self.bus.read_byte(self.ADDRESS)
      value2 = self.bus.read_byte(self.ADDRESS)
      if value1==value2:
        break;
    voltage = value1 / 255.0 * 3.3  #calculate the voltage value
    voltage = round(voltage,2)
    return voltage
  
  def i2cClose(self):
    self.bus.close()
    
def main(args=None):
  rclpy.init(args=args)

  battery = Battery()

  rclpy.spin(battery)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  battery.i2cClose()
  battery.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
