
run `catkin_make` under `catkin_ws` directory to build the package.
 
Follow these two steps to run auction-based task allocation algorithm.

1. generate precedence graph 

   Navigate to `catkin_ws/src/mrta/src/PIA` folder and run `DataGenerator.py 1`. This will generate `data/dataset1.pickle` file.

2. start robots

    run `start_robot.py 1`.

3. allocate tasks

    run `allocate_tasks.py 1`.

To print the result of the PIA algorithm, run `print_results.py 1`. 
