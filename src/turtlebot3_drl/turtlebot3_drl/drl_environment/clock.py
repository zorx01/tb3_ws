import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose
import time

class ObstacleFollower(Node):
    def __init__(self):
        super().__init__('obstacle_follower')
        self.set_entity_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        while not self.set_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def follow_path(self, obstacle_name, keyframes):
        num_keyframes = len(keyframes)
        while True:
            for i in range(num_keyframes):
                current_time, current_position, current_orientation = keyframes[i]
                next_time, next_position, next_orientation = keyframes[(i + 1) % num_keyframes]
                
                # Interpolate between keyframes based on time
                delta_time = next_time - current_time
                start_time = time.time()
                while time.time() - start_time < delta_time:
                    progress = (time.time() - start_time) / delta_time
                    pose = Pose()
                    pose.position.x = current_position[0] + progress * (next_position[0] - current_position[0])
                    pose.position.y = current_position[1] + progress * (next_position[1] - current_position[1])
                    pose.position.z = current_position[2]  # Ignore changes in z-coordinate
                    pose.orientation.x = current_orientation[0] #+ progress * (next_orientation[0] - current_orientation[0])
                    pose.orientation.y = current_orientation[1] #+ progress * (next_orientation[1] - current_orientation[1])
                    pose.orientation.z = current_orientation[2] #+ progress * (next_orientation[2] - current_orientation[2])
                    pose.orientation.w = current_orientation[3] #+ progress * (next_orientation[3] - current_orientation[3])

                    set_state_request = SetEntityState.Request()
                    set_state_request.state.name = obstacle_name
                    set_state_request.state.pose = pose
                    set_state_request.state.reference_frame = 'world'

                    self.set_entity_state_client.call_async(set_state_request)
                    self.get_logger().info(f'Set {obstacle_name} state at time {time.time()}')

def main(args=None):
    rclpy.init(args=args)
    obstacle_follower = ObstacleFollower()
    keyframes = [
        (0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        (10, (-0.5, -1.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        (50, (-3.5, -1.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        (70, (-3.7, -3.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        (90, (-3.5, -1.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        (130, (-0.5, -1.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        (140, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        (160, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    ]
    obstacle_follower.follow_path('turtlebot3_drl_obstacle1', keyframes)
    rclpy.spin(obstacle_follower)
    obstacle_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
