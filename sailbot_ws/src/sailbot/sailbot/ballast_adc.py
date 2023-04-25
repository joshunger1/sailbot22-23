import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int16, Float32
import time
import smbus
import math


class BallastADC(Node):
    def __init__(self):
        super(BallastADC, self).__init__('ballast_adc')

        # info for the Ballast i2c ADC value
        self.bus = smbus.SMBus(0)  # use 0 for older versions of Jetson Nano
        self.address = 0x48  # address of the adc
        self.ballast_adc_publisher = self.create_publisher(Float32, 'ballast_adc_vals', 10)

        # info for the Heading i2c value
        self.heading_adc_publisher = self.create_publisher(Float32, 'heading_adc_publisher', 10)

        timer_period = 0.5  # Fetch data every 1 second
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # some MPU6050 Registers and their Address
        self.Register_A = 0  # Address of Configuration register A
        self.Register_B = 0x01  # Address of configuration register B
        self.Register_mode = 0x02  # Address of mode register

        self.X_axis_H = 0x03  # Address of X-axis MSB data register
        self.Z_axis_H = 0x05  # Address of Z-axis MSB data register
        self.Y_axis_H = 0x07  # Address of Y-axis MSB data register
        self.declination = -0.00669  # define declination angle of location where measurement going to be done
        self.pi = 3.14159265359  # define pi value
        self.Device_Address = 0x1e  # HMC5883L magnetometer device address

    def calcHeading(self):
        self.Magnetometer_Init()  # initialize HMC5883L magnetometer

        # Read Accelerometer raw value
        x = self.read_raw_data(self.X_axis_H)
        z = self.read_raw_data(self.Z_axis_H)
        y = self.read_raw_data(self.Y_axis_H)

        heading = math.atan2(y, x) + self.declination

        # Due to declination check for >360 degree
        if (heading > 2 * self.pi):
            heading = heading - 2 * self.pi

        # check for sign
        if (heading < 0):
            heading = heading + 2 * self.pi

        # convert into angle
        heading_angle = (heading * 180 / self.pi)
        my_float_msg = Float32()
        my_float_msg.data = heading_angle
        self.heading_adc_publisher.publish(my_float_msg)

    def read_raw_data(self, addr):
        # Read raw 16-bit value
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr + 1)

        # concatenate higher and lower value
        value = ((high << 8) | low)

        # to get signed value from module
        if (value > 32768):
            value = value - 65536
        return value

    def Magnetometer_Init(self):
        # write to Configuration Register A
        self.bus.write_byte_data(self.Device_Address, self.Register_A, 0x70)

        # Write to Configuration Register B for gain
        self.bus.write_byte_data(self.Device_Address, self.Register_B, 0xa0)

        # Write to mode Register for selecting mode
        self.bus.write_byte_data(self.Device_Address, self.Register_mode, 0)

    def timer_callback(self):
        self.calcADC()
        self.calcHeading()

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
        relative_position = feedback_voltage / supply_voltage
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
