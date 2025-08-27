# Unitree Workspace (with Git Submodules)

This repository uses Git submodules to include several dependent repositories.

## Submodules

unitree_ros_ws/src/unitree_ros → your fork of unitree_ros

z1_controller → your fork of z1_controller

z1_sdk → your fork of z1_sdk

## How to clone with submodules

'''bash
git clone --recurse-submodules git@github.com:prominent.johnson/unitree_ws.git
'''

## If you already cloned without submodules

'''bash
cd unitree_ws
git submodule update --init --recursive
'''
