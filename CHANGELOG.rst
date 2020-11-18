^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package locomotion_mode
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO
----
* Transpose orientation and position
* Use Map for motors instead of individual objects and a vector
* Put find leg name into function

NICE TO HAVE
------------
* Wait for transition to robot pose to finish
* Create template function for a frequency based execution.
* Remove node options from LocomotionMode
* Change Function Names to camelCase
* robotposes are implemented in as specific deployment and steering positions. Might make sense to generalize them and have a single vector containing position and names (inkl. DEP_RM or STR_LL) to make it more flexible and simplify the code.

KNOWN BUGS
----------
* Currently 'allow_undeclared_parameters(true)' is needed since the pose names are defined in the allow_undeclared_parameters.
Declaring other parameters manually in order to use default values doesn't work atm.
* Enable pose transition does not work at the first mode activation

0.0.2 - (unreleased)
------------------
* Changing modes now stops the driving
* make load_robot_model into bool
* Overwrite dedicated function instead of rover_velocities_callback
* Use Regex to determine leg name
* Use regex to find driving, steering and deployment joints
* Cleanup parameter loading
* Make transpose_pose private
* Reorder definition and declarations
* Remove urdf calls from simple_rover_locomotion and stop_mode
* Add wheel diameter computation to rover class
* Move parsing of URDF into rover class
* Namespaced all files w/ locomotion_mode

0.0.1 - 2020-07-27
------------------
* fixed name of file
* added generic poses that can be used in transitions or otherwise.
* implemented real services for enabling and disabling of locomotion mode
* Added README.
* passing up proper node name from derived class
* Split Locomotion Mode Library from Simple Rover Locomotion
* loading of MaRTA Xacro Model
* reading out limits from urdf file works.
* implemented generic rover library
* added function to find transform of steering in respect to base.
* load paths from config file
* load xacro file
* added launch file
* loaded rover model from config file
* inheritance from locomotion_mode class
