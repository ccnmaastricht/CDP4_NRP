Follow those steps in order to get the CDP4 Loop Experiment running:

1. You will need to install the NRP 3.2 docker image. To do that, download the nrp_installer.sh script from (https://neurorobotics.net/downloads/nrp_installer.sh) and then execute it with the following commands:

	$ ./nrp_installer.sh update
	$ ./nrp_installer.sh install latest

2. Start your NRP Docker containers and access the NRP Frontend through https://localhost:9000/#/esv-web. If it's your first time to access the NRP Frontend, you'll have to provide the following login credentials
        username: nrpuser
        password: password

3. After accessing the NRP Frontend, navigate to the 'My experiments' tab and click on the 'Import folder' button. From there upload the entire cdp4_loop_experiment/ folder. Once the upload finishes, you should see the CDP4_scene_understanding listed under the 'My Experiments' tab. Note that the upload can take a few seconds.

4. Before starting the experiment, some additional models and packages need to be installed. For that you'd have to execute a script inside the NRP Backend Docker container. To do so, please execute the following commands inside a Terminal:

	a. The first step is to access the NRP Backend container
		`$ docker exec -it nrp bash`

	b. Navigate to the directory where the experiment files are located
		`$ cd /home/bbpnrsoa/.opt/nrpStorage/cdp4_loop_experiment_0`

	c. Make the install.sh script executable
		`$ chmod +x install.sh`

	d. Run the install.sh script. This will update apt packages, create a new virtualenv and install some pip packages
		`$ ./install.sh`

5. You can now launch and start the experiment from the Frontend. After starting, some logs will appear in the Frontend's console showing for example the output from the Target Selection module.
