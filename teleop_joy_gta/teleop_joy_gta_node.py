# #!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import threading
import os 
import signal
import time
import math


#>>>>>>>CONFIGURATION>>>>>>> 
def linear(x):
    return x

def two_exponential(x):
    return (2**abs(x) - 1) * math.copysign(1, x)

def log2(x):
    return (math.log2(x) - 1) * math.copysign(1, x)

def x_power_two(x):
    return (x**2) * math.copysign(1, x)

def x_power_three(x):
    return x**3

trigger_curve_dict = {
    "linear": linear,
    "two_exponential": two_exponential,
    "log2": log2,
    "x_power_two": x_power_two,
    "x_power_three": x_power_three,
}
#<<<<<<<CONFIGURATION-SETUP<<<<<<< 

#>>>>>>>CONFIGURATION>>>>>>> 
# When using Ubuntu on Mac --> True
mac_keybind = True

# Set top speed
max_linear_x = float(0.31) #m/s
max_angular_z = float(2 * math.pi) #rad/s

# Set speed curve
trigger_curve = trigger_curve_dict["x_power_three"]
#<<<<<<<CONFIGURATION<<<<<<< 


class TeleopJoy(Node):
    
    def __init__(self,
                 mac_keybind = False):
        super().__init__('teleop_joy_gta_node')
        # Node.get_logger().debug("Debug message")
        
        self.idlepub = True

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Joy, 'joy', self.joy_trans, 10)

        self.zeros_thread = threading.Thread(target=self.publish_zeros)
        self.zeros_thread.start()

        self.mac_keybind = mac_keybind
        self.always_on = True
        self.flag = 1
        print(f"""
    mac_keybind     = {mac_keybind} 
    max_linear_x    = {max_linear_x} m/s
    max_angular_z   = {max_angular_z} rad/s 
    trigger_curve   = {trigger_curve.__name__}""")

    def publish_zeros(self):
        rate = self.create_rate(10)
        while rclpy.ok():
            if self.idlepub:
                twist = Twist()
                twist.linear.x = 0.00
                twist.angular.z = 0.00
                self.cmd_vel_pub.publish(twist)
            rate.sleep()

    def joy_trans(self, data):
        x = float(data.axes[0])
        # y = data.axes[1]

        if self.mac_keybind:
            lt_trigger = (data.axes[2]-1)/2
            rt_trigger = -(data.axes[5]-1)/2
        else:
            lt_trigger = (data.axes[5]-1)/2
            rt_trigger = -(data.axes[4]-1)/2
            
        sum_trigger = (rt_trigger + lt_trigger)

        if data.buttons[0] or self.always_on:  # use self to reference instance variable
            self.idlepub = False  # use self here

            linear_x = trigger_curve(sum_trigger) * max_linear_x
            angular_z = trigger_curve(x) * max_angular_z

            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z

            self.cmd_vel_pub.publish(twist)
        else:
            self.idlepub = True  # use self here


def main(args=None):
    try:

        rclpy.init(args=args)
        teleop_joy_node = TeleopJoy(mac_keybind= mac_keybind)
        rclpy.spin(teleop_joy_node)
        teleop_joy_node.destroy_node()
        rclpy.shutdown()
        
    except KeyboardInterrupt:
        try:
            print("KeyboardInterrupt: juiting..")
            pid = os.getpid()
            os.kill(pid, signal.SIGKILL)
                
        except ProcessLookupError:
            print(f"Process with id {pid} does not exist.")
        except PermissionError:
            print(f"You don't have permission to kill process with id {pid}.")
        time.sleep(0.1)
        exit()


if __name__ == '__main__':
    main()
