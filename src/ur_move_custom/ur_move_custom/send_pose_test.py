import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from common_interfaces_merlab.srv import SendJointTrajectoryPoint
from common_interfaces_merlab.srv import SendJointTrajectory

class SimpleTaskManager(rclpy.node.Node):
    def __init__(self):
        super().__init__("send_pose_test")
        self.cli_move_single_pose = self.create_client(SendJointTrajectoryPoint, 'move_single_pose')
        while not self.cli_move_single_pose.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.cli_move_trajectory = self.create_client(SendJointTrajectory, 'move_trajectory')
        while not self.cli_move_trajectory.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')


        
        self.get_logger().info('Service Active. Initiating')
        self.req_traj_multi_point = SendJointTrajectory.Request()
        self.req_traj_point = SendJointTrajectoryPoint.Request()


    def sendPose(self, goal_pose,time_from_start=4):
        self.req_traj_point.goal_point.positions = goal_pose
        self.req_traj_point.goal_point.time_from_start = Duration(sec=time_from_start,nanosec=0)
        return self.cli_move_single_pose.call_async(self.req_traj_point)

    def sendTrajMultiPoint(self, traj_points,time_from_start=4):
        self.req_traj_multi_point.goal_points.points = [[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0]]
        
        for x in range(len(traj_points)):
            self.req_traj_multi_point.goal_points.points[x].positions = traj_points[x]

        #for x in range(len(traj_points)):
        #    self.req_traj_multi_point.goal_points.points[x].positions = traj_points[x]
        #    self.req_traj_multi_point.goal_points.points[x].time_from_start = Duration(sec=time_from_start,nanosec=0)
        return self.cli_move_multi_pose.call_async(self.req_traj_multi_point)


        
def main():
    rclpy.init()

    task_manager = SimpleTaskManager()
    future = task_manager.sendPose([1.0,0.0,0.0,0.0,0.0,0.0])
    future = task_manager.sendTrajMultiPoint([[1.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0]])
    rclpy.spin_once(task_manager)
    if future.done():
       #Get response
        response = future.result()
        if response.success == True:
            print('Successful operation')
    task_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
