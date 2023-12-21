# Lab 7
## Step 1
Setup and enter the catkin workspace that you cloned the repository into (catkin_ws should be replaced with the name of your workspace).
> `source /opt/ros/noetic/setup.bash`
> `cd catkin_ws`
> `catkin_make`
> `source devel/setup.bash`

## Step 2
To download everything needed for the simulation environment, do the following:
1. Make a directory using the following command:
`mkdir -p ~/ariac_ws/src`
2. Enter the directory you just made (`cd ~/ariac_ws/src`)
3. Clone the repository needed for the simulation environment and install any missing dependencies using the following commands:
`git clone https://github.com/cwru-eecs-373/cwru_ariac_2019.git`
`rosdep install --from-paths ariac --ignore-src -r -y`
4. Build the simulation environment (exit the directory first)
5. Install the simulator environment using the following command:
`sudo -- /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make-DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install"`
6. Add the OSRF repository using the following command:
`sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable bionic main" >/etc/apt/sources.list.d/gazebo-stable.list'`
7. Add cryptographic keys using the following command:
`wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -`
8. Update the apt with the new repository (`sudo apt-get update`)
9. Install a missing operating system dependency necessary for the simulator (`sudo apt install libignition-math4`)
10. Install the ARIAC simulation environment (`sudo apt-get install ariac3`)

## Step 3
In the src of your catkin workspace, do the following:
1. Clone the repository
2. Make a directory using the the following command:
`mkdir -p ~/ecse_373_ariac_ws/src`
3. Enter the directory you just made (`cd ecse_373_ariac_ws/src`)
4. Clone the repository needed for this lab using the following command:
`git clone https://github.com/cwru-eecs-373/ecse_373_ariac.git`
5. Enter the following command to install any missing dependencies:
`rosdep install --from-paths ecse_373_ariac --ignore-src -r -y`
6. Build the catkin_workspace once all above steps are completed

## Step 4
Clone the ik service repository into the src of the catkin workspace (the same place where the ariac entry clone is. A link to the ik service repository is below:
[https://github.com/foodeater87/csds473_f23_msa127_ik_service]

## Step 5
Use the following launch commands to run the program:
1. `roslaunch ariac_entry ariac_entry_launch.launch &`
2. `roslaunch ik_service ik_service.launch &`
3. `roslaunch ariac_entry node_launch.launch &`

## Useful Links
ARIAC 2019 Documentation Page:
[https://bitbucket.org/osrf/ariac/wiki/2019/Home]
