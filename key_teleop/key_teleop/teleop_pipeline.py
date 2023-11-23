import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from autonoma_msgs.msg import VehicleInputs
from autonoma_msgs.msg import PowertrainData

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.subscription = self.create_subscription(
            Int32,
            'keyboard_key',
            self.keypress_callback,
            10)
        self.subscription = self.create_subscription(
            PowertrainData,
            'powertrain_data',
            self.powertrain_callback,
            10)
        self.control_publisher = self.create_publisher(
            VehicleInputs,
            'vehicle_inputs',
            10
        )
        self.throttle_ = 0.0
        self.brake_ = 0.0
        self.steering_ = 0.0
        self.current_gear_ = 1
        self.gear_cmd_ = 0
        self.deltatime_ = 0.025
        self.key_released_ = False
        self.throttle_input = 0
        self.brake_input = 0
        self.steering_input = 0

    def powertrain_callback(self, msg):
        self.current_gear_ = msg.current_gear

    def keypress_callback(self, msg):

        shift_cmd = 0

        if msg.data == 97:
            if self.key_released_:
                shift_cmd = 1
            self.key_released_ = False
        elif msg.data == 121:
            if self.key_released_:
                shift_cmd = -1
            self.key_released_ = False
        elif msg.data == 259:
            self.throttle_input = 1
            self.brake_input = 0
        elif msg.data == 258:
            self.brake_input = 1
            self.throttle_input = 0
        elif msg.data == 260:
            self.steering_input = 1
        elif msg.data == 261:
            self.steering_input = -1
        else:
            self.key_released_ = True
            shift_cmd = 0
            self.throttle_input = 0
            self.brake_input = 0
            self.steering_input = 0
        
        if (self.throttle_input > self.throttle_):
            self.throttle_ = self.rateLimitUpdate(self.throttle_input,self.throttle_,1.5)
        else:
            self.throttle_ = self.rateLimitUpdate(self.throttle_input,self.throttle_,5.0)
        if (self.brake_input > self.brake_):
            self.brake_ = self.rateLimitUpdate(self.brake_input,self.brake_,2.5)
        else:
            self.brake_ = self.rateLimitUpdate(self.brake_input,self.brake_,5.0)

        if (abs(self.steering_input) > abs(self.steering_)):
            self.steering_ = self.rateLimitUpdate(self.steering_input,self.steering_,2.5)
        else:
            self.steering_ = self.rateLimitUpdate(self.steering_input,self.steering_,5.0)

        control_msg = VehicleInputs()

        if (self.current_gear_ + shift_cmd) in range(1,7,1):
            control_msg.gear_cmd = self.current_gear_ + shift_cmd
        else:
            control_msg.gear_cmd = self.current_gear_
        
        control_msg.throttle_cmd = self.throttle_*100.0
        control_msg.brake_cmd = self.brake_*6000.0
        control_msg.steering_cmd = self.steering_*200.0

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