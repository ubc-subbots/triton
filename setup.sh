install_darknet(){
    if [ -d "./darknet" ] 
    then
        echo "Darknet repo already cloned"
    else
        git clone https://github.com/AlexeyAB/darknet
    fi
    cd darknet
    git checkout darknet_yolo_v4_pre
    make -B OPENCV=1 GPU=1 ARCH=-gencode=arch=compute_86,code=sm_86
    rm CMakeLists.txt
    cd ..
}

install_darknet
