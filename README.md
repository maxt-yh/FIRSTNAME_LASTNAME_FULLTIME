# Project Instruction
## QuickStart Guide
The repository has been tested in Ubuntu 18.04 with ROS Melodic.
* Source "setup.bash" at the devel folder of your workspace.
```javascript
source devel setup.bash
```
* Launch planner.launch.
```javascript
roslaunch config planner.launch
```
* Using rqt to call the rosservice "/uapdate_goal"
```javascript
rosrun rqt_service_caller rqt_service_caller
```
* Modify the "Request" of "/uapdate_goal" service data as follow, then click "call"
```javascript
geometry_msgs/PoseStamped start
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
geometry_msgs/PoseStamped goal
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id = "agent_1"
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x = 2
      float64 y = 5
      float64 z = 0
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
float32 tolerance
```

* Modify the "Request" of "/uapdate_goal" service data as follow, then click "call"
```javascript
geometry_msgs/PoseStamped start
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
geometry_msgs/PoseStamped goal
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id = "agent_2"
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x = 6
      float64 y = 3
      float64 z = 0
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
float32 tolerance
```
* The path will be displayed on rviz. Yellow points represent Agent_1 path. Red points represent Agent_2 path,
![alt text](https://github.com/maxt-yh/FIRSTNAME_LASTNAME_FULLTIME/blob/master/config/agent.png "Agent Map")


---
## Algorithm
This project uses A* algorithm for path planning. The algorithm flow is as follows:
```javascript
1. Initial OpenList and CloseList
2. Add the starting point to OpenList and set the priority to 0.
3. If OpenList is not empty, select node N with the highest priority from OpenList.
4. If node N is the end point, then:
   (1) Gradually track the parent node from the end point, until reach the starting point.
   (2) Return the found result path.
5. If node N is not the end point, then:
   (1) Delete node N from OpenList and add it to CloseList.
   (2) Traverse all neighboring nodes of node N:
   (3) If the neighboring node M is in the CloseList, then:
   (4) Skip, select the next neighboring node.
   (5) If the neighboring node M is not in the OpenList, then:
      1) Set the parent of node M to node N.
      2) Compute the priority of node M.
      3) Add node M to OpenList.
   (6) If the neighboring node M is in the OpenList, then:
      1) Set the parent of node M to node N.
      2) Compute the priority of node new M.
      3) Compare new M priority with privious M priority.
      4) Choice the highest priority of M.
      5) Update the parent of node M.
6. Go back to step 3.
```