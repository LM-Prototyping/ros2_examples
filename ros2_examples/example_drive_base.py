from rclpy.node import Node

"""
Set Power Example

Used robot model: CompleteRobot
"""

class DriveExampleNode(Node):
    def __init__(self, name, print_movement=True):
        super().__init__(name)

        if print_movement: 
            print(
                """
                The robot should loop through following movements, for each movement a new random value is selected:
                    - Drive forwards 
                    - Turn left
                    - Drive forwards
                    - Turn left
                    - Turn Right
                    - Drive backwards
                    - Turn right
                    - Turn right
                    - Turn right
                    - Drive forwards
                    - Stop
                """
            )

        self.left_wheel_port = "PORT_A"
        self.right_wheel_port = "PORT_B"

        self.movements = [
            self.forwards,
            self.left,
            self.forwards,
            self.left,
            self.right,
            self.backwards,
            self.right,
            self.right,
            self.right,
            self.forwards,
            self.stop
        ]

        self.counter = 0

        self.timer_period = 2 # seconds
        self.timer = self.create_timer(self.timer_period, self.set_next_movement)

    def set_next_movement(self):
        random_value = self.get_random_movement_value()

        print(f"{self.counter + 1} / {len(self.movements)}")
        self.movements[self.counter](random_value)

        self.counter = (self.counter + 1) % len(self.movements)

    def get_random_movement_value(self):
        raise NotImplementedError()

    def forwards(self, value):
        print("Forwards\n")
        # Left turns counter clockwise (forwards) -> positiv
        # Right turns clockwise (forwards) -> negative
        self.send_new_movement(value, -value)

    def left(self, value):
        print("Left\n")
        # Left turns clockwise (backwards) -> negative
        # Right turns clockwise (forwards) -> negative
        self.send_new_movement(-value, -value)

    def right(self, value):
        print("Right\n")
        # Left turns counter clockwise (forwards) -> positiv
        # Right turns counter clockwise (backwards) -> positiv
        self.send_new_movement(value, value)

    def backwards(self, value):
        print("Backwards\n")
        # Left turns counter clockwise (backwards) -> negativ
        # Right turns clockwise (backwards) -> positiv
        self.send_new_movement(-value, value)

    def stop(self, _):
        print("Stopping\n")
        self.send_new_movement(0, 0)

    def send_new_movement(self, left_val, right_val):
        raise NotImplementedError()