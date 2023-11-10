import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from shapes_interfaces.action import DoCircle, DoPolygon
from shapes_interfaces.msg import Circle, Polygon
import threading


class ShapeMotionController(Node):

    def __init__(self):
        super().__init__('shape_controller')

        self.circle_publisher = self.create_publisher(Circle, 'move_in_circle', 10)
        self.polygon_publisher = self.create_publisher(Polygon, 'move_in_polygon', 10)

        self.do_circle_action_client = ActionClient(self, DoCircle, 'do_circle')
        self.do_polygon_action_client = ActionClient(self, DoPolygon, 'do_polygon')

    def get_circle_input(self):
        while True:
            try:
                radius = float(input('Enter Circle Radius: '))
                speed = float(input('Enter Circle Speed: '))
                direction = int(input('Enter Circle Direction (1 for clockwise, 0 for counterclockwise): '))
                return Circle(radius=radius, speed=speed, direction=direction)
            except ValueError:
                print("Invalid input. Please enter numeric values.")

    def get_polygon_input(self):
        while True:
            try:
                num_vertices = int(input('Enter Number of Vertices: '))
                side_length = float(input('Enter Side Length: '))
                speed = float(input('Enter Polygon Speed: '))
                direction = int(input('Enter Polygon Direction (1 for clockwise, 0 for counterclockwise): '))
                return Polygon(num_vertices=num_vertices, side_length=side_length, speed=speed, direction=direction)
            except ValueError:
                print("Invalid input. Please enter numeric values.")

    def do_circle_action(self, radius, speed, direction):
        goal_msg = DoCircle.Goal()
        goal_msg.command.radius = radius
        goal_msg.command.speed = speed
        goal_msg.command.direction = bool(direction)

        self._send_goal_future = self.do_circle_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def do_polygon_action(self, num_vertices, side_length, speed, direction):
        goal_msg = DoPolygon.Goal()
        goal_msg.command.num_vertices = num_vertices
        goal_msg.command.side_length = side_length
        goal_msg.command.speed = speed
        goal_msg.command.direction = bool(direction)

        self._send_goal_future = self.do_polygon_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.motion_finish}")
    
    def feedback_callback(self, future):
        feedback_msg = future.feedback
        self.get_logger().info(f"Feedback: {feedback_msg.time_left}")

    def run_input_loop(self):
        while True:
            print("Select action:")
            print("1. Publish Circle")
            print("2. Publish Polygon")
            print("3. Execute Circle Action")
            print("4. Execute Polygon Action")
            print("5. Quit")
            choice = input('Enter your choice: ')

            if choice == '1':
                circle_msg = self.get_circle_input()
                self.circle_publisher.publish(circle_msg)
                print("Circle message published.")

            elif choice == '2':
                polygon_msg = self.get_polygon_input()
                self.polygon_publisher.publish(polygon_msg)
                print("Polygon message published.")

            elif choice == '3':
                radius = float(input('Enter Circle Radius: '))
                speed = float(input('Enter Circle Speed: '))
                direction = int(input('Enter Circle Direction (1 for clockwise, 0 for counterclockwise): '))
                self.do_circle_action(radius, speed, direction)

            elif choice == '4':
                num_vertices = int(input('Enter Number of Vertices: '))
                side_length = float(input('Enter Side Length: '))
                speed = float(input('Enter Polygon Speed: '))
                direction = int(input('Enter Polygon Direction (1 for clockwise, 0 for counterclockwise): '))
                self.do_polygon_action(num_vertices, side_length, speed, direction)

            elif choice == '5':
                break

            else:
                print("Invalid choice. Please enter 1, 2, 3, 4, or 5.")

        # Ensure that the action clients are destroyed before shutting down
        self.do_circle_action_client.destroy()
        self.do_polygon_action_client.destroy()

    def run(self):
        input_thread = threading.Thread(target=self.run_input_loop)
        input_thread.start()
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    shape_controller = ShapeMotionController()

    try:
        shape_controller.run()
    finally:
        shape_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
