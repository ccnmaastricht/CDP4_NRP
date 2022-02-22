# This script adds the objects to the local NRP Models and installs
# missing pip packages that are required for the experiment

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

# Download 3d Models if they do not exist in $HBP/Models
if [ ! -d $HBP/Models/FourRooms ]; then
    echo "Download 3d models. This might take some time ..."
    #curl https://neurorobotics-files.net/index.php/s/XrrfZPi4Zstnny8/download --output FourRooms.zip
    curl https://neurorobotics-files.net/index.php/s/g6QA6bpZDMnk4j2/download --output FourRooms.zip
    unzip FourRooms.zip
    rm FourRooms.zip
    mv FourRooms/ $HBP/Models
    $HBP/Models/create-symlinks.sh
fi

# Update apt packages
#curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt upgrade -y --fix-missing

# install virtual coach dependencies
pip install texttable
pip install future
pip install roslibpy

# Create new virtual environment
cd $HOME
virtualenv cdp4_venv
source cdp4_venv/bin/activate
pip install tensorflow==2.2.1
pip install protobuf==3.9.2
deactivate

# Comment out CLE Line to avoid importing Nest
sed -i '44 s/^#*/#/' $HBP/CLE/hbp_nrp_cle/hbp_nrp_cle/brainsim/__init__.py

# Change saliency model owner to be able to delete experiment if needed
sudo chown -R bbpnrsoa:bbp-ext $HOME/.opt/nrpStorage/cdp4_data_collection_experiment_0/resources/

# Install spiking_saccade_generator and gazebo_ros_logical_camera ROS Packages
if [ ! -d $HBP/GazeboRosPackages/src/spiking_saccade_generator ]; then
    cp -r $HOME/.opt/nrpStorage/cdp4_data_collection_experiment_0/resources/spiking_saccade_generator_package $HBP/GazeboRosPackages/src/spiking_saccade_generator
else
    rm -rf $HBP/GazeboRosPackages/src/spiking_saccade_generator
    cp -r $HOME/.opt/nrpStorage/cdp4_data_collection_experiment_0/resources/spiking_saccade_generator_package $HBP/GazeboRosPackages/src/spiking_saccade_generator
fi

if [ ! -d $HBP/GazeboRosPackages/src/gazebo_ros_logical_camera ]; then
    cp -r $HOME/.opt/nrpStorage/cdp4_data_collection_experiment_0/resources/gazebo_ros_logical_camera_package $HBP/GazeboRosPackages/src/gazebo_ros_logical_camera
else
    rm -rf $HBP/GazeboRosPackages/src/gazebo_ros_logical_camera
    cp -r $HOME/.opt/nrpStorage/cdp4_data_collection_experiment_0/resources/gazebo_ros_logical_camera_package $HBP/GazeboRosPackages/src/gazebo_ros_logical_camera
fi

cd $HBP/GazeboRosPackages/
catkin build

# Restart the NRP
sudo supervisorctl restart ros-simulation-factory_app
