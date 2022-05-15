from robotpose import RobotPose
import numpy as np

def mover_robot_a_destino_ctrl(goal_pose):
    initial_pose = RobotPose(0,0,0)
    poses = [RobotPose(0,0,0)] + [RobotPose(*pose) for pose in goal_pose]
    movements = [poses[i+1] - poses[i] for i in range(len(poses)-1)]
    print(movements)

goal_poses = [(1,0, 90), (1,1,180), (0,1,-90), (0,0,0)]
mover_robot_a_destino_ctrl(goal_poses)