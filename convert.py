import rospy
import rospkg
import sys
import rosbag
from sensor_msgs.msg import CameraInfo, Image, Imu
from dvs_msgs.msg import Event, EventArray
from dv import AedatFile
import os
from cv_bridge import CvBridge
import yaml

class Converter:

    def __init__(self, pkg_path, filename):

        if not os.path.exists(pkg_path + '/config/params.yaml'):
            print("Config file not found.\nExiting.")
            sys.exit(1)

        param_file =  open(pkg_path + '/config/params.yaml')
        params = yaml.safe_load(param_file)

        self._filename = filename

        self._in_file = pkg_path + params['aedat4_folder'] + self._filename + '.aedat4'
        self._out_file = pkg_path + params['bag_folder'] + self._filename + '.bag'

        if not os.path.exists(self._in_file):
            print("Source aedat4 file not found.\nExiting.")
            sys.exit(2)

        if not os.path.exists(pkg_path + params['bag_folder']):
            print("Creating folder to contain the ROS bags.")
            os.makedirs(pkg_path + params['bag_folder'])

        #0 for timestamp integration, 1 for max num of events on ROS msgs
        self._conversion_type = params['conversion_type']

        #ROS
        self._cam_info_topic = params['cam_info_topic']
        self._img_topic = params['image_topic']
        self._events_topic = params['events_topic']
        self._imu_topic = params['imu_topic']

        self._dt = params['reset_timestamp']


    def run(self):

        bag = rosbag.Bag(self._out_file, 'w')

        
        with AedatFile(self._in_file) as f:

            height, width = f['events'].size

            if 'imu' in f.names :
                imu_cnt = 0
                for i in f['imu']:
                    imu_cnt +=  1
                    imu_msg = Imu()
                    
                    imu_msg.header.seq = imu_cnt
                    imu_msg.header.stamp = self.toRosTime(i.timestamp)

                    imu_msg.linear_acceleration.x = i.accelerometer[0]
                    imu_msg.linear_acceleration.y = i.accelerometer[1]
                    imu_msg.linear_acceleration.z = i.accelerometer[2]

                    imu_msg.angular_velocity.x = i.gyroscope[0]
                    imu_msg.angular_velocity.y = i.gyroscope[1]
                    imu_msg.angular_velocity.z = i.gyroscope[2]

                    #sensor_msgs MagneticField not implemented
                    #sensor_msgs Temperature not implemented
                    
                    bag.write(self._imu_topic, imu_msg,t=imu_msg.header.stamp)

            if 'events' in f.names :
                ev_cnt = 0
                ev_arr_cnt = 0
                events = []

                for ev in f['events']:
                    if ev_cnt == 0:
                        last_ts = ev.timestamp

                    ev_cnt += 1
                    curr_ts = ev.timestamp

                    ev_msg = Event()

                    ev_msg.x = ev.x
                    ev_msg.y = ev.y
                    ev_msg.ts = self.toRosTime(ev.timestamp)
                    ev_msg.polarity = ev.polarity

                    if self.reset_condition(curr_ts, last_ts): #TODO: reset condition for a fixed number of events
                        ev_arr_msg = EventArray()
                        ev_arr_cnt += 1
                        #append last and publish msg with the last ts
                        events.append(ev_msg)

                        ev_arr_msg.header.seq = ev_arr_cnt
                        ev_arr_msg.header.stamp = self.toRosTime(ev.timestamp) #eventually take mean: (curr_ts-last_ts)/2
                    
                        ev_arr_msg.height = height
                        ev_arr_msg.width = width

                        ev_arr_msg.events = events

                        bag.write(self._events_topic, ev_arr_msg, t=ev_arr_msg.header.stamp)
                        events = []
                        last_ts = ev.timestamp
                       
                    
                    else:
                        events.append(ev_msg)


            if 'frames' in f.names :
                frame_cnt = 0
                for frame in f['frames']:
                    frame_cnt +=  1
                    img_msg = Image()
                    bridge = CvBridge()

                    img_msg.height = height
                    img_msg.width = width
                    
                    img_msg = bridge.cv2_to_imgmsg(frame.image, encoding="passthrough")

                    img_msg.header.seq = frame_cnt
                    img_msg.header.stamp = self.toRosTime(frame.timestamp)

                    bag.write(self._img_topic, img_msg,t=img_msg.header.stamp)

                        

            if 'triggers' in f.names :   
                print('Triggers currently not implemented.')



        bag.close()

    def reset_condition(self, curr, last):
        return True if (curr - last) >= self._dt * 1e3 else False


    def toRosTime(self,timestamp): #timestamp in us
        return rospy.Time.from_sec( timestamp * 1e-6 )

def get_path(script_name):
    return os.path.dirname(os.path.realpath(script_name))


if __name__ == '__main__':


    if len(sys.argv) < 2:
        sys.exit("Usage: python3 converter.py <aedat file name>")

    pkg_path = get_path(sys.argv[0])

    converter = Converter(pkg_path, sys.argv[1])
    converter.run()