from codecs import EncodedFile
import rclpy
from rclpy.node import Node
from pynput import keyboard

from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

class CameraTest(Node):

    def __init__(self):
        super().__init__('camera_test')
        self.publisher_ = self.create_publisher(
            Image,
            '/triton/drivers/front_camera/image_raw',
            10
        )
        self.subscription = self.create_subscription(
            String, 
            '/triton/triton_camera_test/video_source', 
            self.listener_callback, 
            10
        )
        self._start()

        self.get_logger().info('Camera tester succesfully started!')

        # state
        self.pause = False
        self.forward = False
        self.backward = False

        # https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/
        
        timer_period = 0.1  # seconds
        
        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)
            
        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture("/home/jared/Videos/gate2.mp4")
            
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()



    def _start(self):
        """
        Sets up the keyboard listeners
        """
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )
        self.listener.start()

    def _on_press(self, key):
        """
        Handles key presses
        @param key: They character of the key pressed
        """
        if key == keyboard.Key.space:
            self.pause = True
        elif 'char' in dir(key):
            if key.char == 'b':
                self.backward = True
            elif key.char == 'f':
                self.forward = True

    def _on_release(self, key):
        """
        Handles key releases
        @param key: They character of the key released
        """
        self.pause = False
        self.forward = False
        self.backward = False

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = self.cap.read()
            
        if ret == True:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame, 'bgr8'))
        
            # Display the message on the console
            #self.get_logger().info('Publishing video frame')

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video source')
        data_str = str(data)[26:-2]
        self.get_logger().info(data_str)

        if (data_str == '0'):
            self.cap = cv2.VideoCapture(0)
        else:
            self.cap = cv2.VideoCapture(data_str)
    
    



def main(args=None):
    rclpy.init(args=args)

    camera_tester = CameraTest()

    try:
        rclpy.spin(camera_tester)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()