^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package neo_nav2_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2025-06-18)
------------------
* fixing sim time issue, which didn't allow costmap to spawn (`#31 <https://github.com/neobotix/neo_nav2_bringup/issues/31>`_)
* Refactored rviz launch file (`#28 <https://github.com/neobotix/neo_nav2_bringup/issues/28>`_)
  * Refactored rviz launch file
  * Removed contributor tag from package.xml
  * Added the start_rviz node to the navigation_neo.launch.py file and the necessary launch argument, removed changelog
  * Added the missing declare_use_rviz_cmd action in navigation_neo.launch.py
* Contributors: Adarsh Karan Kesavadas Prasanth, Pradheep Krishna
