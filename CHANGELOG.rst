^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package locomotion_mode
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO
----
* Create template function for a frequency based execution.
* Switch to Abstract Model so people don't need to use URDF specific calls incase we want to later switch to the SDF model
* go through TODO's in code
* robotposes are implemented in as specific deployment and steering positions. Might make sense to generalize them and have a single vector containing position and names (inkl. DEP_RM or STR_LL) to make it more flexible and simplify the code.

NICE TO HAVE
------------

KNOWN BUGS
----------
* Enable pose transition does not work at the first mode activation

0.0.1 (unreleased)
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
