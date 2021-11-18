Follow those steps in order to get the CDP4 Loop Experiment running:

1. You will need to install the NRP 3.2 docker image. To do that, download the nrp_installer.sh script from (https://neurorobotics.net/downloads/nrp_installer.sh) and then execute it with the following commands:

	$ ./nrp_installer.sh update
	$ ./nrp_installer.sh install latest

2. Start your NRP Docker containers and access the NRP Frontend through https://localhost:9000/#/esv-web. If it's your first time to access the NRP Frontend, you'll have to provide the following login credentials
        username: nrpuser
        password: password

3. After accessing the NRP Frontend, navigate to the 'My experiments' tab and click on the 'Import folder' button. From there upload the entire cdp4_loop_experiment/ folder. Once the upload finishes, you should see the CDP4_scene_understanding listed under the 'My Experiments' tab. Note that the upload can take a few seconds.

4. Exit the docker container and copy the saliency model from the root of the repository files.
	a. Exit the docker container
		`$ CTRL + D`

	b. Navigate to the root of the repository files and copy the "salmodel" directory to the experiment files inside the docker container
		`$ docker cp salmodel/ nrp:/home/bbpnrsoa/.opt/nrpStorage/cdp4_loop_experiment_0/resources/` 

5. Before starting the experiment, some additional models and packages need to be installed. For that you'd have to execute a script inside the NRP Backend Docker container. To do so, please execute the following commands inside a Terminal:

	a. The first step is to access the NRP Backend container
		`$ docker exec -it nrp bash`

	b. Navigate to the directory where the experiment files are located
		`$ cd /home/bbpnrsoa/.opt/nrpStorage/cdp4_loop_experiment_0`

	c. Make the install.sh script executable
		`$ chmod +x install.sh`

	d. Run the install.sh script. This will update apt packages, create a new virtualenv and install some pip packages
		`$ ./install.sh`

6. (Optional - for testing)  You can now launch and start the experiment from the Frontend, but the transfer function is disabled by default, so you will only see the icub in an empty room. If you want to enable the transfer function for testing purposes, you have to uncomment line # 7 in the bibi_configuration.bibi file. After starting the experiment with the transfer function enabled, some logs will appear in the Frontend's console showing for example the output from the Target Selection module.
Please remember to disable the transfer function again after testing.

7. To run the experiment (spawning room layouts, moving the eyes, and saving data to disk), follow the following steps:

	a. Navigate to the experiment directory inside the docker container
		`$ cd /home/bbpnrsoa/.opt/nrpStorage/cdp4_loop_experiment_0`

        b. (Optional) If you want to record a new dataset, make sure the dataset directory is deleted
		`rm -rf /home/bbpnrsoa/cdp4_dataset`

	   If this directory exists and contains some data, running the experiment will amend the newly generated data to the existing dataset.

	c. Run the run_experiment_script
		`$ python run_experiment.py`

	   This script will launch the experiment, and inside a loop will select a random room/layout pair and spawn the corresponding objects. It will then add the transfer function to the running simulation, and sleep for a certain amount of time. During this sleep period, the experiment will be running with the transfer function and data will be recorded to disk. You can modify this sleep time to increase or decrease the amount of data you want to record from one layout.
           After waking up from the sleep period, the transfer function will be deleted, and the 3d objects will be deleted. Then, a new loop iteration will be started, where another room/layout pair will be selected, and so on.
           The deletion of the transfer function during the spawning and deletion of 3d objects ensures, that data will only be recorded when rooms are fully spawned with objects.

8. (Optional - to test the Saccade Generator separately) If you want to test the Saccade Generator ROS Service outside of the NRP, you first have to start the ROS service inside the docker container. In a terminal, run the following command:
	`$ rosrun spiking_saccade_generator move_eyes.py`

   In another terminal inside the docker container, you can call the rosservice with the desired inputs. For example:
	`$ rosservice call /move_eyes "stim_time: 0.0
                                       stim_duration: 0.0
                                       saccade_size_horizontal: 0.0
                                       saccade_size_vertical: 0.0
                                       last_horizontal: 0.0
                                       last_vertical: 0.0
                                       previous_count: [0, 0, 0, 0]"`

   You can use tab completion when typing the `rosservice call` command, which makes it easier to know what all the required inputs are.

   There is also a python script in `resources/` called `test_saccade_generator.py` that you can run after starting the ROS service. The script runs the Saccade Generator multiple times inside a loop.

   Note: If you modify the Saccade Generator code in `resources/spiking_saccade_generator_package/` you'll have to run the install.sh again to install the new package. Otherwise, your changes will not be taken into consideration.

  
