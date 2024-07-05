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
        self.subscriber_bbox = self.create_subscription(DetectionBoxArray, "/triton/gazebo_drivers/front_camera/bounding_box", self.save_bbox, 10)
        self.lastmsg = None
        self.bbox_queue = [] #List of past images 

    def save_bbox(self, msg: DetectionBoxArray):
        if len(msg.boxes)==0:
            return
        if len(self.bbox_queue)>100: #Keep last 100 bounding box msgs (bounding box is published at render time, image is published after modified)
            self.bbox_queue.pop(0)
        self.bbox_queue.append(msg) #Add timestamp and msg to queue
    
    # def save_image(self, msg: Image):
    #     self.get_logger().info("Getting image...")
    #     if len(self.image_queue)>20: #Keep last 20 iamges
    #         self.image_queue.pop(0)
    #     self.image_queue.append(msg) #Add timestamp and image to queue

    def compare_time(self, bbox_queue, image_time):
        #assumes timestamp of image msg is before timestamp of bbox msg
        selected = None
        for msg in bbox_queue:
            time_msg = msg.header.stamp.sec*1e9+msg.header.stamp.nanosec
            if (time_msg - image_time)<9e6 and (time_msg - image_time)>=0:
                selected = msg
        return selected

    def save_image(self, msg: Image):
        #Look through bbox queue for corresponding bbox
        #Update rate of sensor is set to 50, so we want the bounding box timestamp to be <20ms more than image timestamp 
        time_bbox = msg.header.stamp.sec*1e9+msg.header.stamp.nanosec
        selected_bbox = self.compare_time(self.bbox_queue,time_bbox)
        if selected_bbox is None:
            self.get_logger().info("Corresponding bbox not found.")
            return
        self.get_logger().info("Corresponding bbox found!")
        
        bbox = selected_bbox.boxes[0]
        if (bbox.width <= 0 or bbox.height <= 0):
            self.get_logger().info("Bounding box has zero size.")
            return
        
        try:
            if (self.lastmsg and bbox.x == self.lastmsg.x and bbox.y == self.lastmsg.y): #avoid duplicate images/bounding boxes
                return
            self.lastmsg = bbox

            br = cv_bridge.CvBridge()
            selected_im = br.imgmsg_to_cv2(msg, "passthrough")
            rows,cols,channels = selected_im.shape
           
            centre_x = (bbox.x + bbox.width/2)/cols
            centre_y = (bbox.y + bbox.height/2)/rows
            width = bbox.width/cols
            height = bbox.height/rows

            name = "image"+str(random.randint(0,2**16-1))
            self.get_logger().info("Saving..."+name)
            txt_string = f"{bbox.class_id} {centre_x} {centre_y} {width} {height}"

            data_dir = os.path.join(get_package_share_directory("triton_gazebo"),"data")
            if not os.path.exists(data_dir):
                os.makedirs(data_dir)

            f = open(os.path.join(data_dir, name + ".txt"), "w")
            f.write(txt_string)
            f.close()
            cv2.imwrite(os.path.join(data_dir, name + ".png"), selected_im)

            image_with_box = selected_im.copy()
            image_with_box = cv2.rectangle(image_with_box,(int(bbox.x),int(bbox.y)),(int(bbox.x+bbox.width),int(bbox.y+bbox.height)),(0,0,255),1)
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