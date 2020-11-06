^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package locomotion_mode
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO
----
* Move robot poses to different class or function
* robotposes are implemented in as specific deployment and steering positions. Might make sense to generalize them and have a single vector containing position and names (inkl. DEP_RM or STR_LL) to make it more flexible and simplify the code.
* go through TODO's in code
 * Transpose orientation and position
* Generalize Leg name determination
* Use regex to determine joint type
* Create template function for a frequency based execution.

NICE TO HAVE
------------

KNOWN BUGS
----------
* Enable pose transition does not work at the first mode activation

0.0.2 - (unreleased)
------------------
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
