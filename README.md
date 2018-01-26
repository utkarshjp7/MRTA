
run `catkin_make` under `catkin_ws` directory to build the package.
 
Follow these two steps to run auction-based task allocation algorithm.

1. generate precedence graph 

   Navigate to `catkin_ws/src/mrta/src` folder and run `generate_pgraph.py`. This will generate `/tmp/p_graph.pickle` file.

2. start task allocation 

   `roslaunch multi_robot_2dnav navigation.launch`
