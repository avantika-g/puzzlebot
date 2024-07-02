# PuzzleBot

This code is adapted from here: https://github.com/ZoomLabCMU/puzzlebot as part of the Advanced Agents Robotics Technology lab @ CMU.

## Setup

Create and activate a conda environment called `puzzle`.
```
conda env create -f puzzle_env.yml
conda activate puzzle
```
Install dependencies in the `puzzle` environment.
```
pip install casadi
pip install polytope
pip install pybullet
```

## Robot Control

Calibrate the PuzzleBot code for N robots.
```
source devel/setup.bash
roslaunch puzzlebot_assembly run_calib.launch N:=1
```

Run teleoperative interface code.
```
rosrun puzzlebot_assembly run_keyboard.py 1
```
