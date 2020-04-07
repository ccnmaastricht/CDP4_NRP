# This script adds the objects to the local NRP Models, and copies the cdp4_icub
# experiment to the local storage

# Helper function to create a directory, and overwrite if already exists
create_overwrite() {
    if [ ! -d $1 ]; then
        echo -e "${INFO} Creating directory $1"
        mkdir -p $1
    else
        echo -e "${INFO} Overwriting directory $1"
        rm -rf $1
        mkdir -p $1
    fi
}

# install pip packages
pip install --upgrade pip
pip install onnx tensorflow-addons
if [ ! -d onnx-tensorflow ]; then
    git clone https://github.com/onnx/onnx-tensorflow.git && cd onnx-tensorflow
    pip install -e .
    cd ../
fi

# Copy Models to local NRP
create_overwrite "$HBP/Models/cdp4_objects"
cp -r objects/* $HBP/Models/cdp4_objects
$HBP/Models/create-symlinks.sh

# Copy cdp4 GazeboRosPackages to local NRP
cp -r GazeboRosPackages/cdp4_scene_understanding/ $HBP/GazeboRosPackages/src/

# Copy individual modules to local NRP
cp -r Object_recognition/ $HBP/GazeboRosPackages/src/cdp4_scene_understanding/

# Build local GazeboRosPackages
cd $HBP/GazeboRosPackages/
catkin_make
