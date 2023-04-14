import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int16, Float32
import time
import smbus

class BallastADC(Node):
    def __init__(self):
        super(BallastADC, self).__init__('ballast_adc')

        # info for the Ballast i2c ADC value
        self.bus = smbus.SMBus(0)  # use 0 for older versions of Jetson Nano
        self.address = 0x48  # address of the adc
        self.ballast_adc_publisher = self.create_publisher(Float32, 'ballast_adc_vals', 10)  # Wind direction

        timer_period = 1  # Fetch data every 1 second
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.calcADC()

    def calcADC(self):
        config = [0x40, 0x83]
        self.bus.write_i2c_block_data(self.address, 0x01, config)
        time.sleep(0.1)
        # read the data from the ADC
        data = self.bus.read_i2c_block_data(self.address, 0x00)
        reading1 = (((data[0] << 8) & 0xFF00) + (data[1] & 0x00FF))  # Analog pin from POT

        time.sleep(0.1)
        # convert the data to a voltage
        config = [0x50, 0x83]  # configuration for the ADC
        self.bus.write_i2c_block_data(self.address, 0x01, config)  # tell the ADC to read off of the other pin
        time.sleep(0.1)
        data = self.bus.read_i2c_block_data(self.address, 0x00)
        reading2 = (((data[0] << 8) & 0xFF00) + (data[1] & 0x00FF))  # supply voltage

        feedback_voltage = reading1 * 6.144 / 32767.0
        supply_voltage = reading2 * 6.144 / 32767.0
        relative_position = feedback_voltage/supply_voltage
        my_float_msg = Float32()
        my_float_msg.data = relative_position
        self.ballast_adc_publisher.publish(my_float_msg)


def main(args=None):
    rclpy.init(args=args)

    ballast_adc = BallastADC()

    rclpy.spin(ballast_adc)

    ballast_adc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
