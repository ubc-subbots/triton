install_darknet(){
    if [ -d "./darknet" ] 
    then
        echo "Darknet repo already cloned"
    else
        git clone https://github.com/AlexeyAB/darknet
    fi
    cd darknet
    git checkout aef928cc0c0394f9e07232e037ea191b18ab3431
    make -B OPENCV=1 GPU=1 ARCH=-gencode=arch=compute_86,code=sm_86
    rm CMakeLists.txt
    cd ..
}

install_darknet
