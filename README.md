# jackal_BARN
Repo for working on the [BARN dataset ](https://www.cs.utexas.edu/~xiao/BARN/BARN.html) task 

## Data collection
world_number: 0-299
* rosrun jackal_barn data_collection.py <world_number>

## Visualization
* roslaunch jackal_barn data_visualizer.launch
* rosrun jackal_barn rviz_obs.py <world_number> 
* rosrun jackal_barn rviz_vehicle.py <world_number> 

## Recording obstacles poses
Set get_cylinder_poses to 1 if you want to retrieve the poses of the cylinders (obstacles) from Gazebo. 
* rosrun jackal_barn rviz_obs.py <world_number> <get_cylinder_poses>


