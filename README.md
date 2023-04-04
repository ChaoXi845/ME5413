# README

ME5413_Final_Project_Solution
This is a solution of ME5413 Final Project
Authors: Bo Li, Yuhong Tian, Haoyan Xue and Zongrui Li

## 1  Dependencies

system Requirements:
Ubuntu 20.04
ROS Noetic
C++11 and above
CMake: 3.0.2 and above

Libraries:
       Ceres
       Evo
       Cartographer
       Teb-Local-planner
       

## 2  Installation

Note: First of all, make sure you have installed ROS

### 2.1 Install Cartographer

First, we need to install the wstool and rosdep in order to built Cartographer ROS, and install Ninja for faster builds.
On Ubuntu Focal with ROS Noetic use these commands to install the above tools:
<code>
sudo apt-get update
sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
</code>

After the tools are installed, create a new cartographer_ros workspace in ‘cartographer_ws’:
<code>
mkdir cartographer_ws //create a new folder named cartographer_ws
cd cartographer_ws
wstool init
wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src
</code>

Now you need to install cartographer_ros’ dependencies. First, we use rosdep to install the required packages. 
The command ‘sudo rosdep init’ will print an error if you have already executed it since installing ROS. This error can be ignored.
If the command ‘rosdep install –from-paths src –ignore-src –rosdistro=${ROS_DISTRO} -y’ print the error: ‘
you can delete the line 46 of package.xml: ‘<depend>libabsl-dev</depend>’, the file is in ‘~/cartographer_ws/src/cartographer/’.
<code>
sudo rosdep init
rosdep update
rosdep install –from-paths src –ignore-src –rosdistro=${ROS_DISTRO} -y
</code>

Cartographer uses the abseil-cpp library that needs to be manually installed using this script:
<code>
src/cartographer/scripts/install_abseil.sh
</code>

Build and Install:
<code>
catkin_make_isolated –install –use-ninja
</code>

Now that **Cartographer and Cartographer’s ROS integration** are installed. When you want to run cartographer_ros, you might need to source your ROS environment by running source install_isolated/setup.bash first (replace bash with zsh if your shell is zsh).

### 2.2 Install teb_local_planner

First, we need to install the dependency:
<code>
rosdep install teb_local_planner
</code>

Then, we install the teb_local_planner ROS Package from the github:
<code>
git clone https://github.com/rst-tu-dortmund/teb_local_planner.git
</code>

After installing the package, you can move the folder into the ‘ME5413_Final_Project/src’, ‘ME5413_Final_Project’ is the ROS work space of this solution.

Build and Install:
<code>
catkin_make -DCATKIN_WHITELIST_PACKAGES=’teb_local_planner’
</code>

In the end, you can check the plugin of teb_local_planner compiled completely whether or not. Hope you can find the teb_local_planner!
<code>
source devel/setup.bash // In the ME5413_Final_Project
rospack plugins –attrib=plugin nav_core
</code>

### 2.3 Install Costmap_Prohibition_Layer

For the ME5413_Final_Project, there are some areas which cannot be accessible or be detected by the optics sensor: the operation room and the glass wall(transparent). For these areas, we create an additional costmap to help the global planner while navigation.

First, we can install a plugin from http://github.com/rst-tu-dortmund/costmap_prohibition_layer :
<code>
git clone https://github.com/rst-tu-dortmund/teb_local_planner.git
</code>

After installing the package, you can move the folder into the ‘ME5413_Final_Project/src’, ‘ME5413_Final_Project’ is the ROS work space of this solution. And built the situation:
<code>
catkin_make // In the ME5413_Final_Project 
</code>

In the end, you can check the plugin of Costmap_Prohibition_Layer compiled completely whether or not. If you can find the Costmap_Prohibition_Layer, CONGRATULATION!
<code>
source devel/setup.bash // In the ME5413_Final_Project
rospack plugins –attrib=plugin costmap_2d
</code>

If you have installed all of the tools, you can go to the next step: Usage. 

## 3  Usage

### 3.1 Mapping by Cartographer

During this project, we used both gmapping and cartographer to map the area. We found the map created by cartographer is better, so we just introduce the process of cartographer. We will introduce how to compare the results between two algorithms by evo-tool simply at the end.
 a) Modify in the launch file of me5413_world, the mapping.launch file. Just comment the node that called gmapping before.

 b) Enter the catkin_ws directory of Cartographer and modify the demo_backpack_2d file in install_isolated/share/cartographer_ros/launch. Here you need to modify the command to call the robot. And adjust the parameters of remap according to the sensors on the specific robot. Here I modified scan, imu, points, and odom.

c) Backpack_2d.lua was called in the previous step, and the parameters of the backpack_2d.lua file need to be adjusted here to make the image building effect better. Adjust the following parameters, tracking_frame = "base_link"
published_frame = "odom"
According to the actual scene, the parameters of TRAJECTORY_BUILDER need to be adjusted, and this step can be realized in backpack_2d.lua.
Add to:

```
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 160
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 50.
TRAJECTORY_BUILDER_2D. missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65
```

d) After all configuration files are adjusted, execute in ME5413_Final_Project
<code>
source devel/setup.bash
roslaunch me5413_world world.launch
</code>
Open a new terminal
<code>
source devel/setup.bash
roslaunch me5413_world mapping.launch
</code>
Open Rviz here and you can see that the map topic on the left is not received.
Enter the catkin_ws space of cartographer and execute:
<code>
source install_isolated/setup.bash
ros launch cartographer_ros backpack_2d.launch
</code>
At this time, when you return to Rviz, you can observe that the map nodes have been subscribed, and you can start building maps online. If the cartographer window keeps popping up warning at this time, you can run it again.
<code>
ros launch cartographer_ros backpack_2d.launch
</code>

e) After finishing mapping, run the following command in the thrid terminal to save the map:
<code>
     # Save the map as `my_map` in the `maps/` folder
     roscd me5413_world/maps/
     rosrun map_server map_saver -f my_map map:=/map
</code>
For cartographer, if you want to locate, you need a file of type pbstream, you can open a terminal in the workspace of cartographer and execute.

If you want to use EVO's evaluation tool to evaluate the accuracy of SLAM. A bag file can be recorded during the mapping process.

#View all topics and data formats, using the command:
<code>
Rostopic list -v
</code>

#Find the topic in nav_msgs/Odometry format, and find that Estimated odometry is stored in /odometry/filtered topic

**Record bag file, use the command:**

<code>
rosbag record /gazebo/ground_truth/state /odometry/filtered -o bag_filename.bag
</code>

#Use EVO to evaluate SLAM accuracy, choose to evaluate absolute pose error, and use the command:
<code>
evo_ape bag bagfile_name.bag /gazebo/ground_truth/state /odometry/filtered -a ---plot --plot_mode=xy
</code>

**Finish the first trajectory. No further data will be accepted on it.**

```
rosservice call /finish_trajectory 0
```

**Ask Cartographer to serialize its current state.**

**(press tab to quickly expand the parameter syntax)**

```
rosservice call /write_state "{filename: '${HOME}/Downloads/topics.bag.pbstream', include_unfinished_submaps: "true"}"
```

### 3.2  Modify the AMCL params for better result

a)  Find the file: ‘acml.launch’ in the ‘$(find jackel_navigation)/launch/include’, copy it into the ‘~/ME5413_Final_Project/src/me5413_world/launch’.

b)  Modify the params, the attach code is our params had modified:
<code>
</code>
Of course, you can modify the params to adjust your map and robot.  

### 3.3   Compiling Costmap_Prohibiton_Layer

a)     Find the two files: ‘global_costmap_params.yaml’ and ‘local_costmap_params.yaml’ in the ‘$(find jackal_navigation)/params/map_nav_params’. You can copy the folder ’params’ into the folder ‘ME5413_Final_Project’.

Adding the following code in two ‘.yaml’:

```
# plugins:

      {name: static_layer, type: “costmap_2d::StaticLayer”}
      {name: obstacles_layer, type: “costmap_2d::ObstacleLayer”}
        #added code
      {name:costmap_prohibition_layer, type: “costmap_prohibition_layer_namespace::CostmapProhibitionLayer”}
        #
      {name: inflater_layer, type: “costmap_2d::InflationLayer”}
```

b)     Modifying the ‘move_base.launch’ in the folder ‘~/ME5413_Final_Project/params’:
Adding the following code in the launch:
<code>
<!-- added code -->
<rosparam file=” ~/ME5413_Final_Project/params/map_nav_params/prohibition_areas.yaml” command=”load” ns=”global_costmap/costmap_prohibition_layer” />
<rosparam file=” ~/ME5413_Final_Project/params/map_nav_params/prohibition_areas.yaml” command=”load” ns=”local_costmap/costmap_prohibition_layer” />
<!-- ------ -->
</code>

c)     Create a new .yaml file named ‘prohibition_areas’ in the folder ‘ME5413_Final_project/params’, keep the three .yaml file (global_costmap_params.yaml, local_costmap_params.yaml and prohition_areas.yaml) in the same folder. And edit the prohibition_areas.yaml:
<code>
prohibition_areas:



**point**

[17.09, -6.38]



**line** 

[[8.33, 2.11],
[8.26, 5.11]]

area 

[[-11.15, -15.14],
[-12.35, -13.89],
[-10.05, -12.21]]

one [,] is a point for [x,y], you can measure the point what you want to create the prohibition area in your map, maybe you can use ‘publish point’ tool in the ROS Rviz, you can get the coordination at the bottom side.

### 3.2 Compiling the Teb-Local-Planner

a) Create a new .yaml file named teb_local_planner, you can get a tutorial:
<code>
git clone https://github.com/rst-tu-dortmund/teb_local_planner_tutorials.git
</code>

In the cfg folder, there are three kinds of robot you can choose. For the ME5413_Final_Project, you can choose carlike robot. Copy the teb_local_planner_param.yaml into the ‘~/ME5413_Final_Project/params’.

b) Configurate the teb_local_planner_param.yaml as your situation. There are some important params which may influence your result:
param	impact	Our cfg
Allow_init_with_backwards_motion	Allow the robot plan the path with backwards motion initially	False
Max_vel_x	The max velocity of x axis	0.5
Max_vel_theta	The max angular velocity of z axiz	0.8
Min_turning_radius	The min turning radius of your robot	0.3

When we sought the best planner path, we found these four params were important for my robot. If we set ‘allow_init_with_backwards_motion’ True, the robot will turn around ceaselessly at the global point to adjust its orientation. 
Then the max_vel_x is an important param if you do not want your robot rush into the obstacles_layer or move like a turtle.
Max_vel_theta and min_turning_radius should be coadjust. If your max_vel_theta is low, your min_turning_radius should be tuned larger. Of course, when you tune the params, you should consider the robot condition.

c) Modifying the move_base.launch file:
<code>
<node pkg=”move_base” type=”move_base” respawn=”false” name=”move_base” output=”screen”>
 …
<!-- added code -->
<param name=”base_local_planner” value=”teb_planner/TebLocalPlannerROS” />
<!-- added end -->
<remap from=”odom” to=”odometry/filtered” />
 …
</code>

After all of above, you have walked the path we have spent weeks on! Enjoy your journey.