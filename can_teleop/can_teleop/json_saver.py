import roslibpy as rospy
from a2rl_bs_msgs.msg import VectornavIns
import json 

class JSONSaver:
    def __init__(self, json_file_path, template_json):
        self.json_file_path = json_file_path
        self.template_json = template_json
        self.client = rospy.Ros(host='localhost', port=9090)
        self.client.run()
        self.subscriber = rospy.Topic(self.client, '/a2rl/vn/ins', 'a2rl_bs_msgs/VectornavIns', throttle_rate=100)
        self.subscriber.subscribe(self.save_json)
        
        self.reference = [[]]
        self.speed = []
        
        with open(self.template_json, 'r') as f:
            self.trajectory = json.load(f)
            
        self.trajectory['ReferenceSpeed'] = []
        self.trajectory['ReferenceLine'] = []
        
        try:
            while True:
                pass
        except KeyboardInterrupt:
            self.client.terminate()

    def save_json(self, message):
        self.reference = ([message['position_enu_ins']['x'], message['position_enu_ins']['y'], message['position_enu_ins']['z']])
        speed = message['velocity_enu_ins']['x'] ** 2 + message['velocity_enu_ins']['y'] ** 2
        speed = speed ** 0.5
        # self.speed.append(speed)
        
        self.trajectory['ReferenceSpeed'].append(speed)
        self.trajectory['ReferenceLine'].append(self.reference)     
        
        
    def close(self):
        with open(self.json_file_path, 'w') as f:
            json.dump(self.trajectory, f, indent=4)
        self.client.terminate()

def main():
    json_saver = JSONSaver('/home/humdalab/temp/reference.json', '/home/humdalab/a2rl_base_stack/config/yas_marina_race_line_mue_0_5_4_m_margin.json')
    json_saver.close()

if __name__ == '__main__':
    main()