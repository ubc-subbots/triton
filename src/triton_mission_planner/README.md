# mission_planner

## Description

## Setup

### BehaviorTree Package

The BehaviorTree package can be installed from [here]().

### Architecture

In BehaviorTree, the `.xml` configuration files define the structure of the tree. This is where all of the actions, conditions, and control sequences for the tree are defined. Descriptions of the different node and sequence types can be found in the [BehaviorTree documentation](https://www.behaviortree.dev/docs/category/nodes-library/), or from [this site](https://docs.ros.org/en/indigo/api/behaviortree_cpp_v3/html/namespaceBT.html), which lists all of BehaviorTree's classes. In Triton's case, the tree can be broken down into several [subtrees](https://www.behaviortree.dev/docs/tutorial-basics/tutorial_05_subtrees/), each which focuses on a different task.

Because each custom action that the AUV takes must be its own separate class, these action classes have been grouped together by subtree, as opposed to each getting their own `.hpp` and `.cpp` file.

### Integration With ROS 2

## Usage



## ROS 2 Nodes

- `mission_planner_root` : The mission planner ROS 2 node which coordinates the 

## BehaviorTree Subtrees
  
- `gate_tree` : Contains all of the BehaviorTree nodes for the gate task.
  
- `path_tree` : Contains all of the BehaviorTree nodes for the path tasks.
  
- `buoy_tree` : Contains all of the BehaviorTree nodes for the buoy task.
  
- `bins_tree` : Contains all of the BehaviorTree nodes for the bins task.
  
- `torpedo_tree` : Contains all of the BehaviorTree nodes for the torpedo task.

## Config

- `gate_tree.xml` : Contains the BehviorTree structure for the gate task.

- `path_tree.xml` : Contains the BehviorTree structure for the path task.

- `buoy_tree.xml` : Contains the BehviorTree structure for the buoy task.

- `bins_tree.xml` : Contains the BehviorTree structure for the bins task.

- `torpedo_tree.xml` : Contains the BehviorTree structure for the torpedo task.

## Contributors

- Clark Jeffrey (cjeffreybda@outlook.com)