## Instructions on how to run the CDP4 Toy Experiment inside an NRP Docker container

1. Make sure you have a working NRP Docker container based on the NRP legacy Docker image. Information on how to install an NRP container can be found here (https://neurorobotics.net/local_install.html). Make sure to choose the legacy image, as it's the only Docker image with Python2.7 support, which is used in this experiment.

2. Start your NRP Docker containers and access the NRP Frontend through https://localhost:9000/#/esv-web. If it's your first time to access the NRP Frontend, you'll have to provide the following login credentials
	username: nrpuser
	password: password

3. After accessing the NRP Frontend, navigate to the 'My experiments' tab and click on the 'Import folder' button. From there upload the entire cdp4_toy_experiment/ folder. Once the upload finishes, you should see the CDP4_Toy_Experiment listed under the 'My Experiments' tab.

4. Before starting the experiment, some additional models and packages need to be installed. For that you'd have to execute a script inside the NRP Backend Docker container. To do so, please execute the following commands inside a Terminal:
	
	a. The first step is to access the NRP Backend container
		$ docker exec -it nrp bash

	b. (optional) Some installed packages inside the docker container can be updated (including a lot of ROS packages). These updates can bring stability and bug fixes, so it's recommended but not necessary to do them
		$ sudo apt update
	
	​    $ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
	
	​	$ sudo apt upgrade
	
	c. Once you're inside the container, navigate to the directory where the experiment files are located
		$ cd $HOME/.opt/nrpStorage/

	   This directory includes all cloned NRP experiments in that Docker container. If you only imported/cloned the CDP4 Toy Experiment once, it should be inside the cdp4_toy_experiment_0/ directory. Navigate to the experiment files
		$ cd cdp4_toy_experiment_0/

	d. Make the install.sh script executable
		$ chmod +x install.sh

	e. Run the install.sh script. This will copy some object models needed for the experiment to the NRP models and will install some pip packages 
		$ ./install.sh
	
	f. You can now start the experiment by running the run_experiment.py script
		$ python run_experiment.py
	
	   This Python script will automatically laumch the experiment, spawn some objects, move the robot's eyes and save the images. It takes a couple of minutes to start the experiment and spawn the objects. After that, you can join the experiment from the Frontend to see what is happening. The images are saved in $HOME/.opt/nrpStorage/cdp4_toy_experiment_0/training_data/images.

