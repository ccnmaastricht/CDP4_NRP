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

create_overwrite "$HBP/Models/cdp4_objects"
cp -r objects/* $HBP/Models/cdp4_objects
$HBP/Models/create-symlinks.sh

pip install requests
pip install texttable
