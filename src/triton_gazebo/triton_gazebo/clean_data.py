#!/usr/bin/env python3
# Adapted from https://medium.com/@anirudh.s.chakravarthy/training-yolov3-on-your-custom-dataset-19a1abbdaf09
import glob
import os
import gdown
import requests
import cv2

from ament_index_python.packages import get_package_share_directory

def main(args=None):
    shared_dir = get_package_share_directory("triton_gazebo")
    data_dir = os.path.join(get_package_share_directory("triton_gazebo"),"data")# Path to image directory

    window_name = "Press `d` to delete"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL) 
    for file in glob.iglob(os.path.join(data_dir, '*.png')):  
        if "_box" not in file:
            continue
            
        im_box = cv2.imread(file, cv2.IMREAD_ANYCOLOR)

        title, ext = os.path.splitext(file)
        file_orig = title[:-4]+ext
        file_txt = title[:-4]+'.txt'

        cv2.imshow(window_name,im_box)
        if cv2.waitKey(0)==ord('d'):
            print("Deleting...")
            os.remove(file)
            os.remove(file_orig)
            os.remove(file_txt)


if __name__ == '__main__':
    main()