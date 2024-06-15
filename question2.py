import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math

# Constants for PID control
Kp = 1.0
Ki = 0.1
Kd = 0.05

# Constants for obstacle detection and avoidance
MIN_DISTANCE = 1.0  # Minimum distance to an obstacle (in meters)

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

class TWSBRController:
    def __init__(self):
        rospy.init_node('twsbr_controller', anonymous=True)
        self.pub = rospy.Publisher('robot_command', String, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.obstacle_callback)
        self.pid = PID(Kp, Ki, Kd)
        self.setpoint = 0  # Upright position

    def obstacle_callback(self, data):
        ranges = data.ranges
        if min(ranges) < MIN_DISTANCE:
            self.avoid_obstacle()

    def avoid_obstacle(self):
        # Example action: Stop and turn right 90 degrees
        self.publish_command("stop")
        rospy.sleep(1.0)
        self.publish_command("turn_right")
        rospy.sleep(2.0)
        self.publish_command("go_forward")

    def publish_command(self, command):
        rospy.loginfo(f"Sending command: {command}")
        self.pub.publish(command)

    def balance_robot(self, current_angle):
        correction = self.pid.compute(self.setpoint, current_angle)
        # Apply correction to motors or actuators
        self.publish_command(f"balance {correction}")

    def navigate_to_destination(self, destination):
        # Example navigation using GPS coordinates
        rospy.loginfo(f"Navigating to destination: {destination}")
        # Implement navigation logic here
        rospy.sleep(5.0)  # Simulated navigation time

    def run(self):
        rospy.loginfo("TWSBR Controller started.")
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Simulated current angle (replace with actual sensor data)
            current_angle = math.radians(5)  # Example angle for testing
            self.balance_robot(current_angle)
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = TWSBRController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
