import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

class BatteryMonitor(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'battery_status', 10)
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Pin Definitions Need 5 board pins next to one another
        self.bat_p1 = 37  # 14V pin
        self.bat_p2 = 33  # 13V pin
        self.bat_p3 = 31  # 12V pin
        self.bat_p4 = 29  # 11V pin

        # Pin Setup:
        # Board pin-numbering scheme
        GPIO.setmode(GPIO.BCM)  # Set pins low for high inputs
        GPIO.setup(self.bat_p1, GPIO.IN, initial=GPIO.LOW)	# + 14V
        GPIO.setup(self.bat_p2, GPIO.IN, initial=GPIO.LOW)  # + 13V
        GPIO.setup(self.bat_p3, GPIO.IN, initial=GPIO.LOW)  # + 12V
        GPIO.setup(self.bat_p4, GPIO.IN, initial=GPIO.LOW)  # + 11V

    def timer_callback(self):
        msg = String()
        # #If Else Statements
        if self.bat_p1 == 1:
            msg.data = "+14V you're balling"
        elif self.bat_p2 == 1:
            msg.data = "+13V you're still keefin' Chief"
        elif self.bat_p3 == 1:
            msg.data = "+12V recommend turn back to charge"
        elif self.bat_p4 == 1:
            msg.data = "+11V Turn back Battery in danger"
        elif self.bat_p1 == 0 and self.bat_p2 == 0 and self.bat_p3 == 0 and self.bat_p4 == 0:
            msg.data = "You done fucked up this time"
        else:
            msg.data = "Uh oh a fucky wucky happened"

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    bat_monitor = BatteryMonitor()

    rclpy.spin(bat_monitor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bat_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()