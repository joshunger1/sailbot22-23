import math
from time import time
import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String, Float32, Int8, Int16
from collections import deque
import numpy as np
import time
import smbus


class ControlSystem(Node):  # Gathers data from some nodes and distributes it to others

    def __init__(self):
        super().__init__('control_system')
        # Create subscription to serial_rc topic
        self.serial_rc_subscription = self.create_subscription(
            String,
            'serial_rc',
            self.serial_rc_listener_callback,
            10)
        self.serial_rc_subscription

        # Create subscription to airmar_data
        self.airmar_data_subscription = self.create_subscription(
            String,
            'airmar_data',
            self.airmar_data_listener_callback,
            10)
        self.airmar_data_subscription

        # Create subscription to tt_telemetry
        self.trim_tab_telemetry_subscription = self.create_subscription(
            Float32,
            'tt_telemetry',
            self.trim_tab_telemetry_listener_callback,
            10)
        self.trim_tab_telemetry_subscription

        self.ballast_adc_subscription = self.create_subscription(
            Float32,
            'ballast_adc_vals',
            self.ballast_adc_listener_callback,
            10)
        self.ballast_adc_subscription

        # Create publisher to pwm_control
        self.pwm_control_publisher_ = self.create_publisher(String, 'pwm_control', 10)

        # Create publisher to trim_tab_control
        self.trim_tab_control_publisher_ = self.create_publisher(Int8, 'tt_control', 10)
        self.trim_tab_angle_publisher_ = self.create_publisher(Int16, 'tt_angle', 10)

        # Create publisher to ballast_algorithm_debug
        self.ballast_algorithm_debug_publisher_ = self.create_publisher(String, 'ballast_algorithm_debug', 10)

        # Create instance vars for subscribed topics to update
        self.serial_rc = {}
        self.airmar_data = {}
        self.trim_tab_status = {}
        self.ballast_adc_value = 0.5

        # Create instance var for keeping queue of wind data
        self.lastWinds = []

        # Create instance var for keeping queue of roll data
        self.omega = deque(maxlen=4)
        self.alpha = deque(maxlen=3)
        self.lastRollAngle = deque(maxlen=4)

        # Create global variables for autonomous algorithm
        self.minSailingAngle = np.pi / 4  # minimum sailing angle for the boat
        self.dp_min = -np.cos(self.minSailingAngle)  # minimum dot product for the boat's sailing angle
        """
        # max cross-track error, in meters, that we will allow for the boat when going directly 
        # upwind (from centerline! if you want the boat to wander from -10 to +10m off course, 
        # put in 10!) 
        """
        self.max_cte = 20

        # autonomous global variables, position and velocity of objects
        self.boat = np.array([0, 0])  # boat position
        self.wind = np.array([0, 0])  # wind in x and y velocity (m/s)
        self.windDir = self.wind / np.linalg.norm(self.wind)  # normalize the wind vector

        # need to feed new values
        self.goal = np.array([0, 0])  # goal position

        # information stored on the boat's current heading
        self.onAB = False  # whether the boat is currently trying to sail a course on either side of the no-go zone
        self.onA = False
        self.onB = False

    def calc_heading(self):  # calculate the heading we should follow

        # calculate BG (boat to goal vector)
        BGraw = self.goal - self.boat  # absolute distance to goal
        BG = BGraw / np.linalg.norm(BGraw)  # convert to a unit vector

        # compute the dot product of BG and windDir to see if we can sail directly towards the goal

        Dp = np.dot(BG, self.windDir)

        self.get_logger().info("dot product of direction to goal and the wind is", end=' ')
        self.get_logger().info(Dp)

        if Dp > self.dp_min:
            # if current Dp is less than Dpmin, we can sail directly at the goal. Easy-peasy!

            self.get_logger().info("trying to sail directly at the goal")

            self.onAB = False  # we are NOT sailing on the edge of the no-go zone if we sail directly at the goal
            return BG  # return desired heading

        else:
            # if we can't sail directly at the goal we will have to decide how to tack however, if we're already on
            # an upwind course that's not directly at the goal, we'd like to continue on that course unless our
            # crosstrack error is too high

            self.get_logger().info("we cannot sail directly at the goal - calculating best heading")

            # checking if our cross-track error is too high requires knowing the vectors A and B, so we'll start with
            # that: A and B are the vectors that lie on the edge of the no-go zone, so we'll just rotate the upwind
            # direction by + and - theta, where theta is the minimum sailing angle

            # rotation matrix for minimum sailing angle:

            c, s = np.cos(self.minSailingAngle), np.sin(self.minSailingAngle)

            R = np.array(((c, -s), (s, c)))  # rotation matrices
            R2 = np.array(((c, s), (-s, c)))

            # multiply the matrices by the wind - MUST be the upwind direction! (which is just -windDir)

            A = np.matmul(-self.windDir, R)
            B = np.matmul(-self.windDir, R2)

            # now that we have A and B, we can find which points more in the direction we want to go
            ADBG = np.dot(A, BG)  # dot product tells us which vector is pointing more towards the goal
            BDBG = np.dot(B, BG)

            if not self.onAB:  # if we're not on a heading A or B, but we aren't sailing directly at the goal,
                # we need to start moving on A or B.

                if ADBG > BDBG:  # return whichever heading A or B points more towards the goal
                    self.onAB = True
                    self.onA = True
                    self.onB = False
                    return A
                else:
                    self.onAB = True
                    self.onA = False
                    self.onB = True
                    return B

            else:  # if we're on a heading A or B, we only want to change heading if we've accumulated too much
                # cross-track error

                cte_threshold = np.cos(self.minSailingAngle - np.arcsin(self.max_cte / np.linalg.norm(BGraw)))

                self.get_logger().info("cte_threshold is", end=' ')
                self.get_logger().info(cte_threshold)

                self.get_logger().info("A dot BG is", end=' ')
                self.get_logger().info(ADBG)

                self.get_logger().info("B dot BG is", end=' ')
                self.get_logger().info(BDBG)

                if BDBG > cte_threshold:
                    self.onAB = True
                    self.onA = False
                    self.onB = True
                    return B

                if ADBG > cte_threshold:
                    self.onAB = True
                    self.onA = True
                    self.onB = False
                    return A

                # if neither of the above statements evaluate to true we should just keep following whatever path we
                # were already trying to follow

                if self.onA:
                    self.onAB = True
                    self.onA = True
                    self.onB = False
                    return A
                if self.onB:
                    self.onAB = True
                    self.onA = False
                    self.onB = True
                    return B

        # this should not ever return
        self.get_logger().info("you shouldn't be seeing this")
        return np.array(1, 1)

    def serial_rc_listener_callback(self, msg):
        self.get_logger().info('Received msg: "%s"' % msg.data)
        msg_dict = json.loads(msg.data)
        for i in msg_dict:
            self.serial_rc[i] = msg_dict[i]

    def airmar_data_listener_callback(self, msg):
        self.get_logger().info('Received msg: "%s"' % msg.data)
        msg_dict = json.loads(msg.data)
        for i in msg_dict:
            self.airmar_data[i] = msg_dict[i]

    def ballast_adc_listener_callback(self, msg):
        self.ballast_adc_value = msg.data
        self.get_logger().error('Received msg: "%s"' % msg.data)

    def trim_tab_telemetry_listener_callback(self, msg):
        self.get_logger().info('Received msg: "%s"' % msg.data)

        try:
            self.trim_tab_status['wind_dir'] = msg.data
        except Exception as e:
            self.get_logger().error(str(e))

    def update_winds(self, relative_wind):
        # Check we have new wind
        if len(self.lastWinds) != 0 and relative_wind == self.lastWinds[len(self.lastWinds) - 1]:
            return
            # First add wind to running list
        self.lastWinds.append(float(relative_wind))
        if len(self.lastWinds) > 10:
            self.lastWinds.pop(0)
        # Now find best trim tab state
        smooth_angle = self.median(self.lastWinds)
        return smooth_angle

    def find_trim_tab_state(self, relative_wind):  # five states of trim
        smooth_angle = self.update_winds(relative_wind)
        msg = Int8()
        if 45.0 <= smooth_angle < 135:
            # Max lift port
            msg.data = (0)
        elif 135 <= smooth_angle < 180:
            # Max drag port
            msg.data = (2)
        elif 180 <= smooth_angle < 225:
            # Max drag starboard
            msg.data = (3)
        elif 225 <= smooth_angle < 315:
            # Max lift starboard
            msg.data = (1)
        else:
            # In irons, min lift
            msg.data = (4)

        self.trim_tab_control_publisher_.publish(msg)

    def make_json_string(self, json_msg):
        json_str = json.dumps(json_msg)
        message = String()
        message.data = json_str
        return message

    def median(self, lst):
        n = len(lst)
        s = sorted(lst)
        return (sum(s[n // 2 - 1:n // 2 + 1]) / 2.0, s[n // 2])[n % 2] if n else None

    def ballast_algorithm(self):
        # Check wind angle, then check current tilt of boat, then adjust ballast accordingly
        if len(self.lastWinds) == 0:
            return
        self.lastRollAngle.append(self.airmar_data["roll"])
        smooth_angle = self.median(self.lastWinds)
        ballast_angle = 0
        # self.get_logger().info("roll:" + self.airmar_data["roll"])
        delta = self.airmar_data["roll"] - self.lastRollAngle[-1]

        timeDifference = .5  # hypothetically - see main
        omega_n = delta / timeDifference
        self.omega.append(omega_n)
        alpha_n = self.omega[-1] / timeDifference
        self.alpha.append(alpha_n)
        # -- Logging ----------------
        self.ballast_algorithm_debug_publisher_.publish(
            "omega: " + str(omega_n) + " -- " + "alpha / acceleration: " + str(alpha_n) + "\n")
        # Account for a heavy tilt

        # -----------
        # Starboard tack
        if 0 < smooth_angle <= 180:
            # Go for 20 degrees
            if float(self.airmar_data["roll"]) > -12:  # change to roll acc.
                # ballast_angle = 110
                ballast_angle = omega_n * 2
            elif float(self.airmar_data["roll"]) < -20:  # change to roll acc.
                ballast_angle = 80
                # -----------
        # Port tack
        elif 180 < smooth_angle < 360:
            if float(self.airmar_data["roll"]) < 12:
                ballast_angle = 80
            elif float(self.airmar_data["roll"]) > 20:
                ballast_angle = 110

        ballast_json = {"channel": "12", "angle": ballast_angle}
        self.pwm_control_publisher_.publish(self.make_json_string(ballast_json))


def main(args=None):
    rclpy.init(args=args)

    control_system = ControlSystem()

    while rclpy.ok():

        rclpy.spin_once(control_system, timeout_sec=2)

        if len(control_system.serial_rc) < 2:
            pass  # Don't have rc values
        elif float(control_system.serial_rc["state2"]) > 600:  # in RC
            control_system.get_logger().info("Currently in RC")

            if float(control_system.serial_rc["state1"]) < 400:
                # Manual
                manual_angle = int((float(control_system.serial_rc["manual"]) / 2000) * 100) + 65
                state_msg = Int8()
                state_msg.data = 5
                angle_msg = Int16()
                angle_msg.data = manual_angle
                control_system.trim_tab_control_publisher_.publish(state_msg)
                control_system.trim_tab_angle_publisher_.publish(angle_msg)
            elif "wind-angle-relative" in control_system.airmar_data:
                try:
                    control_system.find_trim_tab_state(control_system.airmar_data["apparentWind"]["direction"])
                except Exception as e:
                    control_system.get_logger().error(str(e))
            else:
                control_system.get_logger().info("No wind angle values")
            if float(control_system.serial_rc["state1"]) < 800:
                ballast_angle = 0
                if control_system.serial_rc["ballast"] > 1200:
                    if control_system.ballast_adc_value > 0.2:
                        ballast_angle = 130
                elif control_system.serial_rc["ballast"] < 800:
                    if control_system.ballast_adc_value < 0.8:
                        ballast_angle = 60
                ballast_json = {"channel": "12", "angle": ballast_angle}
                control_system.pwm_control_publisher_.publish(control_system.make_json_string(ballast_json))
            else:
                control_system.ballast_algorithm()
            rudder_angle = (float(control_system.serial_rc["rudder"]) / 2000 * 90) + 25
            rudder_json = {"channel": "8", "angle": rudder_angle}
            control_system.pwm_control_publisher_.publish(control_system.make_json_string(rudder_json))

        elif float(control_system.serial_rc["state2"]) < 600:
            control_system.get_logger().error("Currently in AUTONOMOUS")

            ballast_adc_val = control_system.ballast_adc_value  # get the saved value
            control_system.get_logger().error(f"Ballast ADC value: {str(ballast_adc_val)}")

            # # Control Trim Tab
            # if "wind-angle-relative" in control_system.airmar_data:
            #     try:
            #         control_system.find_trim_tab_state(control_system.airmar_data["apparentWind"]["direction"])
            #     except Exception as e:
            #         control_system.get_logger().error(str(e))
            # else:
            #     control_system.get_logger().info("No wind angle values")
            #
            # if control_system.airmar_data["Latitude"] or control_system.airmar_data["Longitude"]:
            #     control_system.boat = np.array([control_system.airmar_data["Latitude"], control_system.airmar_data["Longitude"]])
            # else:
            #     control_system.get_logger().error("No GPS Data")
            #
            # curr_wind_value = control_system.update_winds(control_system.airmar_data["apparentWind"]["direction"])
            # curr_heading_value = float(control_system.airmar_data["currentHeading"])
            # true_wind_value = (curr_wind_value + curr_heading_value) % 360
            # wind_cos = math.cos(-true_wind_value)
            # wind_sin = math.sin(-true_wind_value)
            #
            # control_system.wind = np.array([wind_sin, wind_cos])
            #
            # desiredHeading = control_system.calc_heading()

            # # Attempting to read airmar data
            # control_system.get_logger().error("Current Heading: " + control_system.airmar_data["currentHeading"])
            # control_system.get_logger().error("Magnetic Deviation: " + control_system.airmar_data["magnetic-deviation"])
            # control_system.get_logger().error("Magnetic Variation: " + control_system.airmar_data["magnetic-variation"])
            # control_system.get_logger().error("Track Degrees True: " + control_system.airmar_data["track-degrees-true"])
            # control_system.get_logger().error(
            #     "Track Degrees Magnetic: " + control_system.airmar_data["track-degrees-magnetic"])
            # control_system.get_logger().error("Pitch: " + control_system.airmar_data["pitchroll"]["pitch"])
            # control_system.get_logger().error("Roll: " + control_system.airmar_data["pitchroll"]["roll"])
            # # control_system.get_logger().error("Lat: " + control_system.airmar_data["Latitude"])
            # # control_system.get_logger().error("Lat-Dir: " + control_system.airmar_data["Latitude-direction"])
            # # control_system.get_logger().error("Long: " + control_system.airmar_data["Longitude"])
            # # control_system.get_logger().error("Long-Dir: " + control_system.airmar_data["Longitude-direction"])
            # control_system.get_logger().error(
            #     "Apparent Wind Speed: " + control_system.airmar_data["apparentWind"]["speed"])
            # control_system.get_logger().error(
            #     "Apparent Wind Direction: " + control_system.airmar_data["apparentWind"]["direction"])

            # TESTING TRIM TAB

            # start_time = time.time()  # Get the current time
            # while time.time() - start_time < 40:  # Run the loop for 40 seconds
            #     current_time = time.time()  # Get the current time in each iteration
            #     switch_case = int(current_time - start_time) % 4  # Determine which switch case to execute
            #
            #     # Execute the switch case
            #     if switch_case == 0:
            #         control_system.get_logger().error("max lift port")
            #         # begin max lift port
            #         state_msg.data = 0
            #         control_system.trim_tab_control_publisher_.publish(state_msg)
            #     elif switch_case == 1:
            #         control_system.get_logger().error("max lift starboard")
            #         # begin max lift starboard
            #         state_msg.data = 1
            #         control_system.trim_tab_control_publisher_.publish(state_msg)
            #     elif switch_case == 2:
            #         control_system.get_logger().error("max drag port")
            #         # begin max drag port
            #         state_msg.data = 2
            #         control_system.trim_tab_control_publisher_.publish(state_msg)
            #     else:
            #         control_system.get_logger().error("max drag starboard")
            #         # begin max drag starboard
            #         state_msg.data = 3
            #         control_system.trim_tab_control_publisher_.publish(state_msg)
            #
            #     # Wait for the remaining time to ensure that each switch case runs for 10 seconds
            #     remaining_time = 10 - (time.time() - current_time)
            #     if remaining_time > 0:
            #         time.sleep(remaining_time)

            # MIN AND MAX FOR RUDDER IS 106.495 AND 32.92

            # start_time = time.time()  # Get the current time
            # while time.time() - start_time < 10:  # Keep looping until 10 seconds have passed
            #     for i in range(int(106.495 * 100), int(32.92 * 100) - 1, -1):
            #         num = i / 100
            #         rudder_json = {"channel": "8", "angle": float(num)}
            #         control_system.pwm_control_publisher_.publish(control_system.make_json_string(rudder_json))

            # destinations = [(42.277055,-71.799924),(42.276692,-71.799912)]
            # if 'Latitude' in control_system.airmar_data and 'Longitude' in control_system.airmar_data:
            #     try:
            #         if control_system.p2p_alg is None:  # Instantiate new
            #             control_system.p2p_alg = p2p.P2P((float(control_system.airmar_data['Latitude']), float(control_system.airmar_data['Longitude'])), destinations[0])
            #         wind = control_system.update_winds(control_system.airmar_data["apparentWind"]["direction"])
            #         action = control_system.p2p_alg.getAction(wind, float(control_system.airmar_data["magnetic-sensor-heading"]), float(control_system.airmar_data["track-degrees-true"]))
            #         control_system.get_logger().error(str(control_system.p2p_alg.getdistance()))
            #         control_system.get_logger().error(str(action))
            #         if action['status'] == 'DONE':
            #             if control_system.p2p_alg.dest == destinations[0]:
            #                 control_system.p2p_alg = p2p.P2P((control_system.airmar_data['Latitude'], control_system.airmar_data['Longitude']), destinations[1])
            #             else:
            #                 control_system.p2p_alg = p2p.P2P((control_system.airmar_data['Latitude'], control_system.airmar_data['Longitude']), destinations[0])
            #         else:  # We have a non-done action (either trim tab or rudders)
            #             if 'tt-state' in action:
            #                 control_system.trim_tab_control_publisher_.publish(int(action['tt-state']))
            #             elif 'rudder-angle' in action:
            #                 rudder_json = {"channel": "8", "angle": int(action['rudder-angle'])}
            #                 control_system.pwm_control_publisher_.publish(control_system.make_json_string(rudder_json))
            #             control_system.ballast_algorithm()
            #     except Exception as e:
            #         control_system.get_logger().error(str(e))
            # else:
            #     control_system.get_logger().error("No latitude and longitude data")

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_system.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
