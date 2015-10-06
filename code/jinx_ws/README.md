This directory will serve as the ROS workspace that will be portable via GitHub.

A script is included that will setup your ROS environment to include this new workspace in the bash profiles so that it can be reached by the necessary ROS utilities. 

A general explanation of the script execution is as follows:

	-gets location of work space (where this is run from)

	-sources the workspace's setup.bash file for your .bashrc

	-adds the alias for a quick cd into this ROS workspace: jinx-ws

You will need to allow the script to execute before you can actually run it. To do so:
	
	chmod u+x ./jinx-ws-setup.sh

This is assuming you are in the jinx_ws/ directory. Then, when you execute the script from this same location, it will set up your environment for the robot.
