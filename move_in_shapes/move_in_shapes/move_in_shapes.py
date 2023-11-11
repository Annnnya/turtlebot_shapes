import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from shapes_interfaces.msg import Circle, Polygon
from shapes_interfaces.action import DoCircle, DoPolygon
from .helper_classes.shape_classes import CircleMover, PolygonMover
import time

class ShapeMotion(Node):

    def __init__(self):
        super().__init__('shape_publisher')

        self.declare_parameter('namespace', "")
        namespace = self.get_parameter('namespace').value

        self.publisher_ = self.create_publisher(Twist, namespace+'/cmd_vel', 10)
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.move_count = 0

        self.circle_subscription = self.create_subscription(Circle, 'move_in_circle', self.circle_callback, 10)
        self.polygon_subscription = self.create_subscription(Polygon, 'move_in_polygon', self.polygon_callback, 10)

        self.do_circle_action_server = ActionServer(self, DoCircle, 'do_circle', self.do_circle_callback)
        self.do_polygon_action_server = ActionServer(self, DoPolygon, 'do_polygon', self.do_polygon_callback)

        self.current_shape_mover = None
        self.action_in_progress = False

    def timer_callback(self):
        if self.current_shape_mover is not None:
            self.publisher_.publish(self.current_shape_mover.next_command())


    def circle_callback(self, msg):
        self.get_logger().info(f'Received Circle message: Radius={msg.radius}, Speed={msg.speed}, Direction={msg.direction}')
        if self.action_in_progress:
            self.get_logger().info("Cannot perform move in circle request: there is an action in progress. Please abort the action or wait for the action to be finished")
            return
        self.current_shape_mover = CircleMover(msg.radius, self.timer_period, msg.direction, msg.speed)
        self.timer.reset()
        
    def polygon_callback(self, msg):
        self.get_logger().info(f'Received Polygon message: Num Vertices={msg.num_vertices}, Side Length={msg.side_length}, Speed={msg.speed}, Direction={msg.direction}')
        if self.action_in_progress:
            self.get_logger().info("Cannot perform move in polygon request: there is an action in progress. Please abort the action or wait for the action to be finished")
            return
        self.current_shape_mover = PolygonMover(msg.num_vertices, msg.side_length, self.timer_period, msg.direction, msg.speed)
        self.timer.reset()


    def do_circle_callback(self, goal_handle):
        self.timer.cancel()
        self.action_in_progress = True
        self.get_logger().info(f'Received Circle action request: Radius={goal_handle.request.command.radius}, Speed={goal_handle.request.command.speed}, Direction={goal_handle.request.command.direction}')
        self.current_shape_mover = CircleMover(goal_handle.request.command.radius, self.timer_period, \
            goal_handle.request.command.direction, goal_handle.request.command.speed)

        feedback = DoCircle.Feedback()
        self.action_main(goal_handle, feedback)


        self.timer.reset()
        self.action_in_progress = False
        goal_handle.succeed()
        result = DoCircle.Result()
        result.motion_finish = "Moving in circle succesfully finished"
        return result

    def do_polygon_callback(self, goal_handle):
        self.timer.cancel()
        self.action_in_progress = True
        self.get_logger().info(f'Received Polygon action request: Num Vertices={goal_handle.request.command.num_vertices}, Side Length={goal_handle.request.command.side_length}, Speed={goal_handle.request.command.speed}, Direction={goal_handle.request.command.direction}')
        self.current_shape_mover = PolygonMover(goal_handle.request.command.num_vertices, goal_handle.request.command.side_length, \
            self.timer_period, goal_handle.request.command.direction, goal_handle.request.command.speed)

        feedback = DoPolygon.Feedback()
        self.action_main(goal_handle, feedback)

        goal_handle.succeed()
        self.timer.reset()
        self.action_in_progress = False
        result = DoPolygon.Result()
        result.motion_finish = "Moving in polygon succesfully finished"
        return result
    
    def action_main(self, goal_handle, feedback):
        feedback_tracker = 0
        for command in self.current_shape_mover.yield_one_cycle_commands():
            self.publisher_.publish(command)
            if command.angular.z == 0.0 and command.linear.x == 0.0:
                self.get_logger().info("Action finihed")
                self.current_shape_mover = None
                return
            feedback_tracker += 1
            if feedback_tracker > 10:
                time_remaining = self.current_shape_mover.time_until_finish()
                feedback_message = f"{time_remaining} seconds remaining until turtle finishes the action"
                self.get_logger().info(feedback_message)
                feedback.time_left = feedback_message
                goal_handle.publish_feedback(feedback)
                feedback_tracker = 0
            time.sleep(self.timer_period)



def main(args=None):
    rclpy.init(args=args)
    shape_publisher = ShapeMotion()
    
    rclpy.spin(shape_publisher)
    
    shape_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()