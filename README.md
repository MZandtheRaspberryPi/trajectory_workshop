# trajectory_workshop
Workshop on trajectory following for legged robots. This README has instructions on setting up your computer to run the simulation. It also has the questions for the lab activities. It finally has some brief notes on running your code on the real robots. Though the teaching assistants will be the ones that facilitate with that, your code will need to have a certain structure that is elaborated on.

## Environment Setup

These are instructions to setup the software on your computer needed to complete the lab. Windows, Mac, and Unix systems should work.

1. Install Python3.12 on your computer. This version of Python is supported on Windows, Mac, and various Linux distributions. Find it [here](https://www.python.org/downloads/).
2. Install Git on your computer, supported on Windows, Mac, and various Linux distributions. Link [here](https://git-scm.com/install/mac).
3. Download the code that we have written for this lab, by opening a terminal (on windows use command prompt) and changing to a known path on your computer you can access (the command `cd` changes the directory, so `cd C:\Users\michael\Documents` changes me to my documents folder in Windows as my username is Michael). "Clone" the repository, meaning download it by running this command in the terminal in the directory you've chosen: `git clone https://github.com/MZandtheRaspberryPi/trajectory_workshop.git`.
4. Create a virtual environment for this lab (a copy of python with the libraries we need) by moving into the directory you've cloned `cd trajectory_workshop` and running the command `python -m venv venv`. Activate the environment, on windows `venv\Scripts\activate` and on Mac/Unix `source venv/bin/activate`. This will use the virutal environment we've created for any python invocations.
5. Install dependencies of this lab: `pip install -e .`.
6. (optional) Install an interactive development environment of your choice to write and run code. This is optional as you can use a text editor like notepad on Windows or the inbuilt text-editor on Mac/Unix.
7. Run the `main.py` by running `python main.py`. A graph should pop up similar to the below figure. This graph may be closed by clicking the X in the top right. Upon closing a file will be created in the directory `traj_data` with the same data that was on the plot. Ensure this all works.

This figure shows in the top left the XY trajectory of the robot along with the goal. Units are in meters. In the middle column, the commands are plotted in blue with the red dotted lines indicating the boundaries of commands above or below which the command will be capped or floored. The third column indicates the trajectory over time, and includes the robot's heading (which is important with the unicycle model as the robot moves in the direction it is facing).  

![traj](./traj_data/traj_2026-01-21_09-04-16.png)

## Labratory Questions

This is the lab content with guided questions on the activities and trajectories.

### Question 1 -- Becoming Familliar with the Simulation
Run `main.py`. Why does the path change each time I run it if i am always passing the same comand? (Hint: look at the intiailization variable `noise_cfg`) and how it is used in the `step` function. What are some scenarios where a robot in the real world might not move exactly forward when commanded to?

Inside of `main.py` the function `get_cmd` is called to get the current velocity command for the robot. `main.py` starts with a goal in x and y of a line from (0, 0) to (1, 1). With reference to the unicycle model, the current implementation of `get_cmd`, and the goal, why does the robot not track the trajectory closely?

If there was no noise in this simulation and the robot starts at an x and y of (0, 0) and with a goal trajectory of a line that starts there and ends at (1, 0) and this trajectory takes 1 second, what would be the velocity command to send to the robot? Change the `LineTraj` in `main.py` to have this goal, change `get_cmd` to implement your given velocity command, and change the `noise_cfg` to `NO_NOISE` in the initialization of `UnicycleSim`. Re-run and ensure success in the output plot. Revert the codechanges you made.

### Question 2 -- Implementing a Controller

Inside of `main.py` there is a section marked `your code starts here` and one that is marked `your code ends here` within the function `get_cmd`. Implement this to implement a controller for the robot to track the initial line trajectory (start (0,0) end (1, 1), seconds 2.0). Show the plot to a Teaching Assistant and give your code to them by email it to them (paste the function into an email).

Now change the trajectory to a circular trajectory. An example is given in `main.py` though this is commented out in the initial file. Implement a controller for this shape.


## Real Robot Setup

This is what the students need to submit to the teaching assistants to run their code on the real robot.


