import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from humda_common.msg import ControlSignal

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.subscription = self.create_subscription(
            Int32,
            'keyboard_key',
            self.keypress_callback,
            10)
        self.control_publisher = self.create_publisher(
            ControlSignal,
            'teleop_output',
            10
        )
        self.throttle_ = 0.0
        self.brake_ = 0.0
        self.steering_ = 0.0
        self.deltatime_ = 0.025

    def keypress_callback(self, msg):
        throttle_input = 1 if msg.data == 259 else 0
        brake_input = 1 if msg.data == 258 else 0
        steering_input = 0.0
        if msg.data == 260:
            steering_input = 1
        elif msg.data == 261:
            steering_input = -1
        else:
            steering_input = 0
        if (throttle_input > self.throttle_):
            self.throttle_ = self.rateLimitUpdate(throttle_input,self.throttle_,1.5)
        else:
            self.throttle_ = self.rateLimitUpdate(throttle_input,self.throttle_,5.0)
        if (brake_input > self.brake_):
            self.brake_ = self.rateLimitUpdate(brake_input,self.brake_,3.5)
        else:
            self.brake_ = self.rateLimitUpdate(brake_input,self.brake_,5.0)

        if (abs(steering_input) > abs(self.steering_)):
            self.steering_ = self.rateLimitUpdate(steering_input,self.steering_,3.5)
        else:
            self.steering_ = self.rateLimitUpdate(steering_input,self.steering_,5.0)

        control_msg = ControlSignal()

        if self.throttle_ > 0:
            control_msg.pedal_position = float(self.throttle_)
        elif self.brake_ >0:
            control_msg.pedal_position = -1.0 * float(self.brake_)
        else:
            control_msg.pedal_position = 0.0
        
        control_msg.steering_angle = float(self.steering_)

        self.control_publisher.publish(control_msg)

    def rateLimitUpdate(self, yCur, yPrev, speed):

        yLimited = 0.0
        if ( abs(yCur-yPrev)/self.deltatime_ > speed ):
            yLimited = yPrev + speed*self.deltatime_ if yCur>yPrev else yPrev - speed*self.deltatime_
        else:
            yLimited = yCur
    
        return yLimited
    
def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()