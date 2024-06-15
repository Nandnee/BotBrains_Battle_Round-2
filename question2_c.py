import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
import speech_recognition as sr
import pyttsx3

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

        # Initialize speech recognition and synthesis
        self.recognizer = sr.Recognizer()
        self.engine = pyttsx3.init()

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
        rospy.loginfo(f"Navigating to destination: {destination}")
        # Implement navigation logic here
        rospy.sleep(5.0)  # Simulated navigation time

    def speech_recognition(self):
        with sr.Microphone() as source:
            rospy.loginfo("Listening for commands...")
            audio = self.recognizer.listen(source)

        try:
            command = self.recognizer.recognize_google(audio)
            rospy.loginfo(f"Recognized command: {command}")
            return command.lower()
        except sr.UnknownValueError:
            rospy.logwarn("Could not understand audio")
            return None
        except sr.RequestError as e:
            rospy.logerr(f"Speech recognition request failed: {e}")
            return None

    def speech_synthesis(self, text):
        self.engine.say(text)
        self.engine.runAndWait()

    def run(self):
        rospy.loginfo("TWSBR Controller started.")
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Simulated current angle (replace with actual sensor data)
            current_angle = math.radians(5)  # Example angle for testing
            self.balance_robot(current_angle)

            # Check for speech command
            command = self.speech_recognition()
            if command:
                if "stop" in command:
                    self.publish_command("stop")
                    self.speech_synthesis("Stopping")
                elif "deliver" in command:
                    destination = "John's house"  # Example destination
                    self.navigate_to_destination(destination)
                    self.speech_synthesis(f"Delivering parcel to {destination}")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = TWSBRController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
