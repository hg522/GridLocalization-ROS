# GridLocalization-ROS
Grid localization of a robot using Bayes Filter and visualization on rviz

GRID LOCALIZATION

Grid Localization is a variant of discrete Bayes Localization. In this project, the map is an occupancy grid. At each time step, the algorithm finds out the probabilities of the robot presence at every grid cell. The grid cells with maximum probabilities at each
step, characterize the robot's trajectory. Grid Localization runs in two iterative steps | Movement and Observation.
After each movement, we compute if that movement can move the robot between grid cells. For each observation, we find the most probable cells where that measurement could have occurred.

The Rosbag file provided includes the movements and observation information from one scenario. the file is read to simulate the   localization algorithm running on a robot executing this scenario.

The following Models (Motion Model, Observation Model and Map) are used in this project:

There are six landmarks in the robot map, and they are at the following locations:
_ Tag 0: x=1.25m, y=5.25m
_ Tag 1: x=1.25m, y=3.25m
_ Tag 2: x=1.25m, y=1.25m
_ Tag 3: x=4.25m, y=1.25m
_ Tag 4: x=4.25m, y=3.25m
_ Tag 5: x=4.25m, y=5.25m

The robot moves in the area within these landmarks, observing some at any given time. A grid is made for say 7m*7m coverage and the cell size can be 20cm*20cm. A dimension is also taken for the robot's heading. So the grid is 3 dimensional. The third dimension covers the robot's heading. It can be discretized with a value like (10 degree, 20 degree or more). The motion model of this robot is (rotation, translation, rota-tion) and the observation model is (range, bearing). In Grid Localization, for the purpose of moving robot between cells, the motion and observation noise is adjusted. A translation and rotation noise is added for movement and range, and bearing noise for the
observation. A good selection for this purpose is half the cell size. So for the example of 20*20 cells and 90 degree discretization, range and translation noises are 10cm, and bearing and rotation noises are 45 degrees.


