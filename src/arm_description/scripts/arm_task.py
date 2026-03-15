#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Path
from spatialmath import SE3
import roboticstoolbox as rtb
import numpy as np

L0 = 0.1
L1 = 0.6
L2 = 0.8

tria = np.array([
    [0.0, 0.9, 0.2],   # A
    [-0.4, 0.3, 0.2],  # B
    [0.4, 0.3, 0.2]    # C
])

rec = np.array([
    [-0.6, 0.6, 0.2],  # A
    [0.6, 0.6, 0.2],   # B
    [0.6, 0.3, 0.2],    # C
    [-0.6, 0.3, 0.2],  # D
])

class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_node')
        self.dt = 1/20
        self.path = Path()
        self.point_index = 0
        self.traj_step = 0
        self.trajectory = None
        self.trajectory_computing = False
        self.target_pos = None
        self.traj_joint_pos = None
        self.traj_joint_vel = None

        self.declare_parameter('task', 'none')
        self.task = self.get_parameter('task').value  
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # joint limits
        qlim = np.array([[-3.5,3.5]])
        # contruct the robot using DH parameters
        T1 = rtb.RevoluteDH(d=L0, a=0.0, alpha=np.pi/2, offset=0.0, qlim=qlim)
        T2 = rtb.RevoluteDH(d=0.0, a=L1, alpha=0.0, offset=0.0, qlim=qlim)
        T3 = rtb.RevoluteDH(d=0.0, a=0.0, alpha=np.pi/2, offset=np.pi/2, qlim=qlim)
        T4 = rtb.RevoluteDH(d=L2, a=0.0, alpha=-np.pi/2, offset=0.0, qlim=qlim)
        T5 = rtb.RevoluteDH(d=0.0, a=0.0, alpha=np.pi/2, offset=0.0, qlim=qlim)
        T6 = rtb.RevoluteDH(d=0.0, a=0.0, alpha=0.0, offset=0.0, qlim=qlim)

        self.robot = rtb.DHRobot([T1, T2, T3, T4, T5, T6], name='my_robot')
        self.q = np.array([1.5, 1.5, -1.5, 0.0, 0.0, 0.0])  # home configuration
        self.q_dot = np.zeros(6)                            # joint velocities

        self.create_timer(self.dt, self.timer_callback)
        self.create_timer(0.1, self.publish_path)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)  
        self.path_publisher = self.create_publisher(Path, 'path', 10)
        self.get_logger().info("Robot initialized.")


    def parameters_callback(self, parameter_list):
        self.path.poses.clear() 
        valid_task = ['tria', 'rec']
        
        for param in parameter_list:
            if param.name == 'task':
                new_task = param.value
                if new_task in valid_task:
                    self.task = new_task
                    self.get_logger().info(f"Task changed to: {new_task}")
                    return SetParametersResult(successful=True)
                else:
                    self.get_logger().warn(f"Invalid task parameter: {new_task}. Valid options are: {valid_task}")
                    return SetParametersResult(successful=False)
        
        return SetParametersResult(successful=True)

    def timer_callback(self):
        next_point = (self.point_index + 1) % len(tria) if self.task == 'tria' else (self.point_index + 1) % len(rec)
        task = tria if self.task == 'tria' else rec

        if self.trajectory is None and not self.trajectory_computing:
            self.trajectory_computing = True
            try:
                # Get SE3 poses for start and goal
                T_start = self.robot.fkine(self.q)
                T_goal = SE3(task[next_point])
                
                # Generate Cartesian trajectory (50 poses interpolated with cubic polynomial)
                cartesian_traj = rtb.ctraj(T_start, T_goal, 50)
                
                # Solve IK for each Cartesian pose
                joint_trajectory = []
                current_q = self.q.copy()
                
                for T_des in cartesian_traj:
                    ik_sol = self.robot.ikine_LM(T_des, q0=current_q)
                    if ik_sol.success:
                        joint_trajectory.append(ik_sol.q)
                        current_q = ik_sol.q
                    else:
                        self.get_logger().warn(f"IK failed at step")
                        joint_trajectory.append(current_q)
                
                self.traj_joint_pos = np.array(joint_trajectory)
                self.traj_joint_vel = np.gradient(self.traj_joint_pos, axis=0) / self.dt
                self.target_pos = task[next_point]
                self.traj_step = 0
                self.trajectory = True
                self.trajectory_computing = False
                # self.get_logger().info(f"Cartesian trajectory computed (point {self.point_index} → {next_point})")
           
            except Exception as e:
                self.get_logger().error(f"Error computing trajectory: {e}")
                self.trajectory_computing = False
                return
        
        if self.traj_step < len(self.traj_joint_pos):
            self.q = self.traj_joint_pos[self.traj_step]
            self.q_dot = self.traj_joint_vel[self.traj_step]
            self.traj_step += 1
        else:
            fk_pose = self.robot.fkine(self.q)
            position_error = np.linalg.norm(self.target_pos - fk_pose.t)
        
            if position_error < 0.02:
                self.trajectory = None
                self.point_index = next_point

        self.publish_joint_states()


    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        msg.position = self.q.tolist()
        msg.velocity = self.q_dot.tolist()
        self.joint_state_publisher.publish(msg)


    def publish_path(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = f"{self.get_namespace().strip('/')}/base_link"

        end_effector_pose = self.robot.fkine(self.q)
        pose_msg.pose.position.x = end_effector_pose.t[0]
        pose_msg.pose.position.y = end_effector_pose.t[1]
        pose_msg.pose.position.z = end_effector_pose.t[2]
        
        self.path.poses.append(pose_msg)
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.header.frame_id = pose_msg.header.frame_id
        self.path_publisher.publish(self.path)
        

def main(args=None):
    rclpy.init(args=args)
    node = ArmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
