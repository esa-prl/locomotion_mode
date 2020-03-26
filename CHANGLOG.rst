^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package locomotion_mode
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO
----
* Add README.
* go through TODO's in code
* implement real services for activation and deactivation of locomotion mode

NICE TO HAVE
------------

KNOWN BUGS
----------
* changing speed ratio while driving does not update joint velocities correctly
* robot_state_publisher does not publish tf frames even though it is receiving joint states.
* Message Definitions don't compile with Header variable...
* robot_state_publisher still prints messages after removing output='screen' in launch file.

0.0.1 (unreleased)
------------------
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
