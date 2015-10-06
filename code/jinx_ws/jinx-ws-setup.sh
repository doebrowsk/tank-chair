#! /bin/bash

dir=$PWD
echo "Got current directory to configure workspace: $dir"
echo
source_loc=$dir/devel/setup.bash
echo "This is the setup.bash file that will be used: $source_loc"
echo

echo "Sourcing the setup.bash file now."
echo "source $source_loc" >> ~/.bashrc
echo

echo "Assigning the alias jinx-ws to take you to the jinx_ws/src directory."
echo "alias jinx-ws=\"cd $dir/src\"" >> ~/.bashrc
echo

echo "Finished. You will need to open a new terminal to have these changes take effect."
echo "To check the alias, try typing in \"jinx-ws\"."
echo "Type \"printenv \| grep ROS\" to see if the new workspace was added to the package path."
