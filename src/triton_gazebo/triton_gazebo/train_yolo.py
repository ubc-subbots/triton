#!/usr/bin/env python3
# Adapted from https://medium.com/@anirudh.s.chakravarthy/training-yolov3-on-your-custom-dataset-19a1abbdaf09
import glob
import os
import gdown
import requests

from ament_index_python.packages import get_package_share_directory

def main(args=None):
    shared_dir = get_package_share_directory("triton_gazebo")
    data_dir = os.path.join(get_package_share_directory("triton_gazebo"),"data")# Path to image directory
    backup_dir = os.path.join(get_package_share_directory("triton_gazebo"),"backup")# Path to backup directory
    darknet_exec = "darknet/darknet"# Path to darknet installation

    if not os.path.exists(backup_dir):
        os.makedirs(backup_dir)
    # Percentage of images to be used for the valid set
    percentage_test = 10
    # Create train.txt and valid.txt
    file_train = open(os.path.join(data_dir,'train.txt'), 'w')  
    file_test = open(os.path.join(data_dir,'valid.txt'), 'w')  
    # Populate train.txt and valid.txt
    counter = 1  
    index_test = round(100 / percentage_test)  
    for file in glob.iglob(os.path.join(data_dir, '*.png')):  
        if "_box" in file:
            continue
        title, ext = os.path.splitext(os.path.basename(file))
        if counter == index_test:
            counter = 1
            file_test.write(data_dir + "/" + title + '.png' + "\n")
        else:
            file_train.write(data_dir + "/" + title + '.png' + "\n")
            counter = counter + 1

    file_train.close()
    file_test.close()

    file_obj_names = open(os.path.join(data_dir,'obj.names'), 'w')  
    file_obj_names.write("Lenna\n") #Put class names here, one on each line
    file_obj_names.close()

    file_obj_data = open(os.path.join(data_dir,'obj.data'), 'w')  
    file_obj_data.write(f"classes = 1\n\
    train  = {os.path.join(data_dir,'train.txt')}\n\
    valid  = {os.path.join(data_dir,'valid.txt')}\n\
    names = {os.path.join(data_dir,'obj.names')}\n\
    backup = {backup_dir}\n")
    file_obj_data.close()


    #gdown.download("https://drive.google.com/uc?id=18v36esoXCh-PsOKwyP2GWrpYDptDY8Zf", os.path.join(shared_dir,'yolov3-tiny.conv.11'))
    if not os.path.exists(os.path.join(shared_dir,'yolov4-tiny.conv.29')):
        with open(os.path.join(shared_dir,'yolov4-tiny.conv.29'),'wb') as f:
            f.write(requests.get('https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.conv.29').content)

    os.system(f"{darknet_exec} detector train {os.path.join(data_dir,'obj.data')} {os.path.join(shared_dir,'config','yolov4-custom.cfg')} {os.path.join(shared_dir,'yolov4-tiny.conv.29')} -map")
    os.system(f"cp {os.path.join(backup_dir,'yolov4-custom_final.weights')} {get_package_share_directory('triton_object_recognition')}")
    os.system(f"cp {os.path.join(shared_dir,'config','yolov4-custom.cfg')} {get_package_share_directory('triton_object_recognition')}")


if __name__ == '__main__':
    main()