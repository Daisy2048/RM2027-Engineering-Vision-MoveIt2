Panthera HT + BIT Vision - Humble MoveIt Servo continuous demo
=============================================================
Required: Ubuntu 22.04, ROS2 Humble, ros-humble-moveit-servo, Panthera Demo,
          BIT_RM_eng_vision built, engineer_vision_tf_bridge built.
WARNING: SIMULATION ONLY. Launch refuses if controller is not FakeSystem
         mock_components/GenericSystem or planner_bridge is running.

1) First terminal:
cd ~/Panthera-HT_ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch panthera_ht_config demo.launch.py

2) Stop previous planner_bridge / planning_trigger / old BIT demo; do not run
   hardware.launch.py or a physical controller. Extract this zip into
   ~/rm2027/engineer_moveit_ws/demos

3) Second terminal:
cd ~/rm2027/engineer_moveit_ws
source /opt/ros/humble/setup.bash
source ~/Panthera-HT_ROS2/install/setup.bash
source ~/rm2027/BIT_RM_eng_vision/install/setup.bash
source install/setup.bash
ros2 launch demos/vision_servo.launch.py

4) Third terminal, after Servo is ready; confirm FakeSystem one more time:
source /opt/ros/humble/setup.bash
ros2 control list_hardware_components
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger '{}'

5) Observe continuous vision targets and feedback (no Enter required):
ros2 topic hz /vision/servo_target_pose_base
ros2 topic hz /servo_node/delta_twist_cmds
ros2 topic echo /servo_node/status --once
ros2 topic echo /joint_states --once

6) Frame-by-frame VALID PnP logs (raw XYZ/q and mapped XYZ):
~/rm2027/engineer_moveit_ws/logs/servo_demo/frames_*.csv

Only VALID PnP frames can be logged; this does not claim every video frame
produces a valid detection. The source orientation is logged but NOT servoed.
These simulated camera extrinsics and demo workspace mapping are NOT suitable
for real ore exchange or real motors. No per-frame OMPL replanning is done:
MoveIt Servo processes 50Hz streaming velocity commands, updated whenever
new vision poses arrive.

If the arm doesn't move: check the Servo node output, status topic, TF
base_link->bat_center, /arm_controller/joint_trajectory, and /joint_states.
If missing, stop via Ctrl+C. No automatic physical safety guarantee.
