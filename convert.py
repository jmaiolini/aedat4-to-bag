import rospy
import rospkg
import sys
import rosbag
from sensor_msgs.msg import CameraInfo, Image, Imu
from geometry_msgs.msg import PoseStamped
from dvs_msgs.msg import EventArray
from dv import AedatFile
import os
import cv2
from cv_bridge import CvBridge
import numpy as np
import yaml

class Converter:

    def __init__(self, _filename):

        param_file =  open(os.getcwd() + '/config/params.yaml')
        params = yaml.safe_load(param_file)
        self._filename = _filename

        self._in_file = os.getcwd() + params['aedat4_folder'] + _filename + '.aedat4'
        self._out_file = os.getcwd() + params['bag_folder'] + _filename + '.bag'

        #0 for timestamp integration, 1 for max num of events on ROS msgs
        self._conversion_type = params['conversion_type']

        #ROS
        self._cam_info_topic = params['cam_info_topic']
        self._img_topic = params['image_topic']
        self._events_topic = params['events_topic']
        self._imu_topic = params['imu_topic']

        self.dt = params['reset_timestamp']


    def run(self):

        bag = rosbag.Bag(self._out_file, 'w')

        
        with AedatFile(self._in_file) as f:

            # if 'events' in f.names :
            #     for e in f['events']:
            #         self.ts2rosTime(e.timestamp)
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

                # img = np.zeros((height, width,3), np.uint8)

                # count = 0
                # temp_img = img.copy()
                # loop through the "events" stream
                # for e in f['events']:

                #     if count == accumulator_size:
                #         temp_img = img.copy()
                #         count = 0
                
                #     if e.polarity == True:
                #         temp_img[e.y,e.x] = (0,255,0)
                #     else:
                #         temp_img[e.y,e.x] = (0,0,255)
                #     cv2.imshow('Event image', temp_img)
                #     cv2.waitKey(1)
                #     count = count + 1

        bag.close()


    def toRosTime(self,timestamp): #timestamp in us
        return rospy.Time.from_sec( timestamp * 1e-6 )


if __name__ == '__main__':


    if len(sys.argv) < 2:
        sys.exit("Usage: python3 converter.py <aedat file name>")

    converter = Converter(sys.argv[1])
    converter.run()