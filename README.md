# MobileRobot-RVO-Obstacle-Avoidance
This repo relate to implementing c++/python code of new obstacle avoidance algorithm based on Reciprocal Velocity Obstacle on Gazebo simulator and ROS1. 
We used at least two turtlebot3 burger models in Gazebo simulator.
---
If you like the project give me a star! :star: 

You can see the video: &nbsp;&nbsp;
[![website](./img/youtube-dark.svg)](https://www.youtube.com/channel/UCyRBig4xgAdaRdIz14Xymrg)
&nbsp;&nbsp;
---

### This package checked on below systems :
- Ubuntu Focal
- ROS Noetic 
- Gazebo11 & TurtleBot3
### Runing process:
1- Frist step is designing Custome Close area in gazebo. This model is in "multi-agents/gazebomodels/CloseArea2"
2- We also run the amcl and move_base node file to recieve and guid the robot to goal point.
![ezgif com-gif-maker](https://user-images.githubusercontent.com/79801785/169010016-f5fa9dcc-67bb-47a9-b1ef-6b3dc3f32603.gif)
