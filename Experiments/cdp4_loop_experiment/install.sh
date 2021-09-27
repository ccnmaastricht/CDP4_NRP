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

#create_overwrite "$HBP/Models/cdp4_objects"
#cp -r objects/* $HBP/Models/cdp4_objects
#$HBP/Models/create-symlinks.sh

# Update apt packages
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt upgrade -y

# Create new virtual environment
cd $HOME
virtualenv cdp4_venv
source cdp4_venv/bin/activate
pip install tensorflow
pip install protobuf==3.9.2
deactivate

# Copy Target Selection module to the virtual environment to be able to import it
cp /home/bbpnrsoa/.opt/nrpStorage/cdp4_loop_experiment_0/model_TS.py /home/bbpnrsoa/cdp4_venv/lib/python3.8/site-packages/
