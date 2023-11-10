from geometry_msgs.msg import Twist
from math import pi

class ShapeMover:
    def __init__(self, timer_period, direction, speed) -> None:
        self.timer_period = timer_period
        self.direction = direction
        self.speed = speed


class CircleMover(ShapeMover):
    def __init__(self, radius, timer_period, direction, speed) -> None:
        super().__init__(timer_period, direction, speed)
        self.radius = radius

        self.angular_speed = self.speed / self.radius
        self.move_count = 0
        self.full_circle_moves = int(1 / self.timer_period * 2 * pi * self.radius / self.speed)

    def next_command(self):
        move_cmd = Twist()
        move_cmd.linear.x = self.speed
        move_cmd.angular.z = self.angular_speed if self.direction else -self.angular_speed
        self.move_count += 1
        return move_cmd
    
    def yield_one_cycle_commands(self):
        for _ in range(self.full_circle_moves):
            yield self.next_command()
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        yield move_cmd
    
    def time_until_finish(self):
        return (self.full_circle_moves - (self.move_count % self.full_circle_moves) ) * self.timer_period


class PolygonMover(ShapeMover):
    def __init__(self, num_vertices, side_length, timer_period, direction, speed) -> None:
        super().__init__(timer_period, direction, speed)
        self.num_vertices = num_vertices
        self.side_length = side_length

        self.angle = 2 * pi / self.num_vertices
        self.move_count = 0
        self.moves_per_side = int(1 / self.timer_period * self.side_length / self.speed)
        self.moves_per_angle = int(1 / self.timer_period * self.angle / (pi / 2)) 
        self.full_cycle_moves = self.num_vertices * self.moves_per_side + self.num_vertices * self.moves_per_angle 
        
    def next_command(self):
        move_cmd = Twist()
        if self.move_count % (self.moves_per_side + self.moves_per_angle) < self.moves_per_side:
            move_cmd.linear.x = self.speed
        else:
            move_cmd.angular.z = (pi / 2) if self.direction else -(pi / 2)
        self.move_count += 1
        return move_cmd

    def yield_one_cycle_commands(self):
        for _ in range(self.full_cycle_moves):
            yield self.next_command()
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        yield move_cmd
    
    def time_until_finish(self):
        return (self.full_cycle_moves - (self.move_count % self.full_cycle_moves)) * self.timer_period


        