#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from triton_interfaces.msg import DetectionBoxArray
from cv2 import cv2
import cv_bridge
import numpy as np
import random
from ament_index_python.packages import get_package_share_directory

class BoundingBoxImageSaver(Node):

    def __init__(self):
        super().__init__('bounding_box_image_saver')
        self.subscribe_image = self.create_subscription(Image, "/triton/drivers/front_camera/image_raw", self.save_image, 10)
        self.subscriber_bbox = self.create_subscription(DetectionBoxArray, "/triton/gazebo_drivers/repub/bounding_box", self.save_bbox, 10)
        self.lastmsg = None
        self.image_queue = [] #List of past images 

    def save_image(self, msg: Image):
        self.get_logger().info("Getting image...")
        if len(self.image_queue)>20: #Keep last 20 iamges
            self.image_queue.pop(0)
        self.image_queue.append(msg) #Add timestamp and image to queue

    def save_bbox(self, msg: DetectionBoxArray):
        if len(msg.boxes)==0:
            return

        #Look through image queue for corresponding image
        #Update rate of sensor is set to 50, so we want the bounding box timestamp to be <20ms more than image timestamp 
        selected = None
        time_bbox = msg.header.stamp.sec*1e9+msg.header.stamp.nanosec
        for im_msg in self.image_queue:
            time_im = im_msg.header.stamp.sec*1e9+im_msg.header.stamp.nanosec
            if (time_bbox - time_im)<9e6 and (time_bbox - time_im)>=0:
                selected = im_msg
        if selected is None:
            self.get_logger().info("Corresponding image not found.")
            return
        self.get_logger().info("Corresponding image found!")
        
        msg = msg.boxes[0]
        if (msg.width <= 0 or msg.height <= 0):
            self.get_logger().info("Bounding box has zero size.")
            return
        
        try:
            if (self.lastmsg and msg.x == self.lastmsg.x and msg.y == self.lastmsg.y): #avoid duplicate images/bounding boxes
                return
            self.lastmsg = msg

            br = cv_bridge.CvBridge()
            selected_im = br.imgmsg_to_cv2(selected, "passthrough")
            rows,cols,channels = selected_im.shape
           
            centre_x = (msg.x + msg.width/2)/cols
            centre_y = (msg.y + msg.height/2)/rows
            width = msg.width/cols
            height = msg.height/rows

            name = "image"+str(random.randint(0,2**16-1))
            self.get_logger().info("Saving..."+name)
            txt_string = f"{msg.class_id} {centre_x} {centre_y} {width} {height}"

            data_dir = os.path.join(get_package_share_directory("triton_gazebo"),"data")
            if not os.path.exists(data_dir):
                os.makedirs(data_dir)

            f = open(os.path.join(data_dir, name + ".txt"), "w")
            f.write(txt_string)
            f.close()
            cv2.imwrite(os.path.join(data_dir, name + ".png"), selected_im)

            image_with_box = selected_im.copy()
            image_with_box = cv2.rectangle(image_with_box,(int(msg.x),int(msg.y)),(int(msg.x+msg.width),int(msg.y+msg.height)),(0,0,255),1)
            cv2.imwrite(os.path.join(data_dir, name + "_box" + ".png"), image_with_box)
        except AttributeError as e:
            self.get_logger().info("No image yet.")
            pass
        

def main(args=None):
    rclpy.init(args=args)
    subscriber = BoundingBoxImageSaver()
    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()