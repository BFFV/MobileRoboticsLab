from math import atan2

# Auxiliary vector functions
def vector_norm(x, y):
    return (x ** 2 + y ** 2) ** 0.5

def dot_product(x0, y0, x1, y1):
    return x0 * x1 + y0 * y1

def vector_angle(x0, y0, x1, y1):
    dot = dot_product(x0, y0, x1, y1)
    det = x0 * y1 - y0 * x1
    return atan2(det, dot)

# Represents a pose for the robot
class RobotPose:
    """Represents a pose and provides operations between them"""
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def linear_distance_to(self, other):
        return vector_norm(other.x - self.x, other.y - self.y)

    def angular_distance_to(self, other):
        return vector_angle(self.x, self.y, other.x, other.y)

    def __add__(self, other):
        return RobotPose(
            self.x + other.x,
            self.y + other.y,
            self.theta + other.theta
        )

    def __sub__(self, other):
        return RobotPose(
            self.x - other.x,
            self.y - other.y,
            self.theta - other.theta
        )

    def __repr__(self):
        return f'<{self.x}, {self.y}, {self.theta}>'