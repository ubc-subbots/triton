install_darknet(){
    if [ -d "./darknet" ] 
    then
        echo "Darknet repo already cloned"
    else
        git clone https://github.com/AlexeyAB/darknet/
    fi
    cd darknet
    make -B OPENCV=1
    rm CMakeLists.txt
    cd ..
}

install_darknet