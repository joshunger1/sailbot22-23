import sys
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class NeonOrangeDetectionNode(Node):
    def __init__(self):
        super().__init__('neon_orange_detection_node')
        self.publisher_ = self.create_publisher(Image, 'neon_orange_detection', 10)
        self.bridge = CvBridge()

        # Define the lower and upper bounds of the neon orange color in HSV format
        self.orange_lower = (0, 90, 80)
        self.orange_upper = (20, 255, 255)

        # Set the minimum area requirement for an object to be considered
        self.min_area = 7500

        # Open the ZED2 camera
        self.cap = cv2.VideoCapture(cv2.CAP_OPENNI2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def run(self):
        while rclpy.ok():
            # Read a frame from the camera
            ret, frame = self.cap.read()

            # Convert the frame to the HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Create a mask for the neon orange color and apply it to the original frame
            neon_orange_mask = cv2.inRange(hsv, self.orange_lower, self.orange_upper)

            # Find contours in the mask
            contours, _ = cv2.findContours(neon_orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Draw a bounding box around the largest contour (if one exists and its area is greater than min_area)
            if len(contours) > 0:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                if area > self.min_area:
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    self.publisher_.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))
                else:
                    self.publisher_.publish(self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

            # Wait for a key press and check if the user pressed the 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Release the camera
        self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    node = NeonOrangeDetectionNode()
    node.run()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
