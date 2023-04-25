import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
import cv2
import numpy as np

class OrangeBlobDetector(Node):

    def __init__(self):
        super().__init__('orange_blob_detector')
        self.publisher_ = self.create_publisher(Int8, 'object_detected', 10)
        self.subscription_ = self.create_subscription(Image, 'camera/image_raw', self.detect_orange_blob, 10)
        self.subscription_  # prevent unused variable warning

        # Define the lower and upper bounds of the neon orange color in HSV format
        self.orange_lower = (0, 90, 80)
        self.orange_upper = (20, 255, 255)

        # Set the minimum area requirement for an object to be considered
        self.min_area = 7500

        # Create a window to display the camera view and the neon orange blob detection with a bounding box
        cv2.namedWindow('Camera View and Neon Orange Blob Detection', cv2.WINDOW_NORMAL)

        # Open the default camera
        self.cap = cv2.VideoCapture("/dev/video0")

        # Set the resolution of the camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def detect_orange_blob(self, msg):
        # Convert the ROS image message to an OpenCV image
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

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
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                # Publish a message indicating that an object was detected
                self.publisher_.publish(Int8(data=1))
            else:
                # Publish a message indicating that no object was detected
                self.publisher_.publish(Int8(data=0))
        else:
            # Publish a message indicating that no object was detected
            self.publisher_.publish(Int8(data=0))

        # Display the camera view and the neon orange blob detection with a bounding box
        cv2.imshow('Camera View and Neon Orange Blob Detection', frame)
        cv2.waitKey(1)

    def __del__(self):
        # Release the camera and close the window
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)

    orange_blob_detector = OrangeBlobDetector()

    rclpy.spin(orange_blob_detector)

    orange
