import can
import cantools
import threading
import os
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from cantools.database import Database
from can.message import Message
from eav24_bsu_msgs.msg import HL_Msg_01, HL_Msg_02
from sensor_msgs.msg import Joy
from enum import Enum
from ament_index_python import get_package_share_directory

class CANDriverManager:
    def __init__(self, bus_name, dbc_file_path):
        self.bus = can.interface.Bus(bus_name, bustype='socketcan')
        self.db = cantools.database.load_file(dbc_file_path)
        self.lock = threading.Lock()

    def get_bus(self):
        return self.bus

    def get_db(self):
        return self.db

    def acquire_lock(self):
        self.lock.acquire()

    def release_lock(self):
        self.lock.release()
        
class ButtonState(Enum):
    RELEASED = 0
    PRESSED = 1
    HOLD = 2
        
class CANDriverSender(Node):
    def __init__(self, manager):
        super().__init__('joy_can')
        self.manager = manager
        self.alive = 0
        self.brake_scale = 50
        self.brake_offset = 50
        self.brake_bias = 0.6
        self.throttle_scale = 50
        self.throttle_offset = 50
        self.gear = 1
        self.steering_scale = 263
        self.steering_offset = 0
        
        self.steering_axis = 0
        self.throttle_axis = 2
        self.brake_axis = 3
        
        self.shift_up_btn_state = ButtonState.RELEASED
        self.shift_down_btn_state = ButtonState.RELEASED
        self.shift_up_btn = 4
        self.shift_down_btn = 5
        
        self.shift_up_press_time = self.get_clock().now()
        self.shift_down_press_time = self.get_clock().now()
        
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.hl_01_pub = self.create_publisher(HL_Msg_01, 'a2rl/eav24_bsu/hl_msg_01',10)
        self.hl_02_pub = self.create_publisher(HL_Msg_02, 'a2rl/eav24_bsu/hl_msg_02',10)

    # Create the needed ros messages subs
    
    def joy_callback(self, ros_msg:Joy):
        db = self.manager.get_db()
        hl01_msg = db.get_message_by_frame_id(257)
        hl02_msg = db.get_message_by_frame_id(258)
        
        brake_pressure_front = ros_msg.axes[self.brake_axis] * self.brake_scale + self.brake_offset
        np.clip(brake_pressure_front, 0, 100)
        brake_pressure_rear = brake_pressure_front * (1 - self.brake_bias) / self.brake_bias
        np.clip(brake_pressure_rear, 0, 100)
        steering = ros_msg.axes[self.steering_axis] * self.steering_scale + self.steering_offset
        if steering >= 100.0:
            steering = 100.0
        elif steering <= -100:
            steering = -100
        throttle = ros_msg.axes[self.throttle_axis] * self.throttle_scale + self.throttle_offset
        
        self.gear = self.gear_box(ros_msg.buttons[self.shift_up_btn], ros_msg.buttons[self.shift_down_btn])

        hl_01_msg:HL_Msg_01 = HL_Msg_01()

        hl_01_msg.hl_targetpressure_rr = brake_pressure_rear;
        hl_01_msg.hl_targetpressure_rl = brake_pressure_rear;
        hl_01_msg.hl_targetpressure_fr = brake_pressure_front;
        hl_01_msg.hl_targetpressure_fl = brake_pressure_front;
        hl_01_msg.hl_target_throttle = throttle;
        hl_01_msg.hl_target_gear = self.gear;
        hl01_msg.hl_alive_01 = (self.alive + 1) % 16

        hl_02_msg:HL_Msg_02 = HL_Msg_02()

        hl_02_msg.hl_alive_02 = (self.alive + 1) % 16
        hl_02_msg.hl_target_psa_control = steering
        hl_02_msg.hl_psa_mode_of_operation = 1
        hl_02_msg.hl_psa_profile_acc_rad_s2 = 1000
        hl_02_msg.hl_psa_profile_dec_rad_s2 = 1000
        hl_02_msg.hl_psa_profile_vel_rad_s = 1000

        self.hl_01_pub.publish(hl_01_msg)
        self.hl_02_pub.publish(hl_02_msg)

        # Fill the signals from ros message
        hl_01_signals = {
            "HL_TargetThrottle": throttle,
            "HL_TargetGear": self.gear,
            "HL_TargetPressure_RR": brake_pressure_rear,
            "HL_TargetPressure_RL": brake_pressure_rear,
            "HL_TargetPressure_FR": brake_pressure_front,
            "HL_TargetPressure_FL": brake_pressure_front,
            "HL_Alive_01": (self.alive + 1) % 16, 
        }
        hl_02_signals = {
            "HL_Alive_02": (self.alive + 1) % 16,
            "HL_TargetPSAControl": steering,
            "HL_PSA_ModeOfOperation": 1,
            "HL_PSA_Profile_Vel_rad_s": 1000,
            "HL_PSA_Profile_Dec_rad_s2": 1000,
            "HL_PSA_Profile_Acc_rad_s2": 1000,
        }
        
        # Create data from signals
        data = hl01_msg.encode(hl_01_signals)
        
        # Send to the can bus
        # self.send_to_can(hl01_msg.frame_id, data)
        
        data = hl02_msg.encode(hl_02_signals)
        # self.send_to_can(hl02_msg.frame_id, data)
        pass

    def send_to_can(self, msg_id, data):
        # self.manager.acquire_lock()  # Acquire the lock from the manager
        try:
            bus = self.manager.get_bus()
            msg = can.Message(arbitration_id=msg_id, data=data, is_extended_id=False)
            bus.send(msg)
        finally:
            # self.manager.release_lock()  # Release the lock
            pass
        
    def gear_box(self, shift_up, shift_down):
        target_gear = self.gear
        if shift_up == 1 and self.shift_up_btn_state == ButtonState.RELEASED:
            self.shift_up_btn_state = ButtonState.PRESSED
            self.shift_up_press_time = self.get_clock().now()
        elif shift_up == 0 and self.shift_up_btn_state == ButtonState.PRESSED:
            self.shift_up_btn_state = ButtonState.RELEASED
            time_diff = (self.get_clock().now() - self.shift_up_press_time).nanoseconds / 1e9
            if time_diff > 0.05:
                target_gear = min(self.gear + 1, 6)
        
        if shift_down == 1 and self.shift_down_btn_state == ButtonState.RELEASED:
            self.shift_down_btn_state = ButtonState.PRESSED
            self.shift_down_press_time = self.get_clock().now()
        elif shift_down == 0 and self.shift_down_btn_state == ButtonState.PRESSED:
            self.shift_down_btn_state = ButtonState.RELEASED
            time_diff = (self.get_clock().now() - self.shift_down_press_time).nanoseconds / 1e9
            if time_diff > 0.05:
                target_gear = max(self.gear - 1, 1)
        return target_gear
        
        
def main(args=None):
    rclpy.init(args=args)

    bus_name = 'vcan0'
    dbc_file_path = os.path.join(get_package_share_directory('can_teleop'), 'EAV24_CAN2.dbc')
    manager = CANDriverManager(bus_name, dbc_file_path)
    node = CANDriverSender(manager)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()