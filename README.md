# PF Localisation Package (Skeleton Code)

## Particle Filter Exercise

This package implements the particle filter localisation using sensor and motion update from the Pioneer P3-DX robot. The methods in `src/pf_localisation/pf.py` have to be completed correctly to run the node. Read the assignment lab notes for more instructions on how to complete these methods. You can also find documentation regarding each method in the source files.

#### Note:

* You need to make changes **ONLY** in `pf.py` file for completing the localisation package. If you want to change any of the inherited parameters (parameters inherited from the parent class `PFLocaliserBase`), it is best to do so from the child class itself (i.e. in `PFLocaliser` class in `pf.py`).

* However, you may play with different values for parameters in the other files (eg. `sensor_model.py`) for conducting experiments.


### Building Package:

* Move package to your colcon workspace (`src` directory)
* Rebuild colcon workspace 
        
        colcon build    # ----- run from root directory of your colcon workspace

### Running the node:

The localisation node can be tested in the Stage simulator.

        ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p node_names:=[map_server] -p autostart:=true
        ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=<path_to_map_file>
        ros2 run stage_ros2 stage_ros2 --ros-args -p world_file:=./src/socspioneer/data/meeting.world
        ros2 launch socspioneer keyboard_teleop.launch.py  # ---- run only if you want to move robot using keyboard 
        ros2 run pf_localisation node.py    # ----- requires completed pf.py methods.

For the node to start publishing you must set an initial pose estimate, for example, through RViz2.

### Published Topics:

Running the node successfully will publish the following topics:

* `/amcl_pose` 
* `/estimatedpose`
* `/particlecloud`

All of these can be visualised in RViz by adding the appropriate Views.


### Advice:

* Once you have a basic understanding of what you have to do in the `PFLocaliser` class of `pf.py` file, it is a good idea to look at its parent class `PFLocaliserBase` in `pf_base.py` to see how the other parts of the localisation algorithm is implemented. 
* You may also go through `sensor_model.py` to see how the sensor model is coded, and how the model update is implemented. The different parameter values that were empirically chosen can also be found here. 
* Read through `node.py` to understand how the node is impelemented to perform the update whenever new information is available. You can also see how `rclpy logging` is used for logging useful information to console. You may also use this for debugging purposes.
