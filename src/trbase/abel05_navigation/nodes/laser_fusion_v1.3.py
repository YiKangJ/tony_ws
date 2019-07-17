#!/usr/bin/env python
# -*- coding=utf-8 -*-

'''
 v1.2 简化
'''

import rospy
from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import PointStamped
import tf

class LaserFusion(object):
    def __init__(self):
        
        self.debug = rospy.get_param('~debug', False)
        
        if self.debug:
            rospy.init_node('laser_fusion_node', log_level=rospy.DEBUG, anonymous=True)
        else:
            rospy.init_node('laser_fusion_node', anonymous=True)

        # 深度图像坐标系
        self.depth_camera_frame = rospy.get_param('~depth_camera_frame', 'camera_link')
        # 雷达坐标系
        self.lidar_frame = rospy.get_param('~lidar_frame', 'laser_link')
        
        # odom里程计坐标系
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        
        # 检测到障碍物激光数据维持时间
        self.duration = rospy.get_param('~duration', 10)
        
        self.laser_from_lidar = None
        self.laser_from_camera = None
        
        # 判断是否相机检测到障碍，update_data>10 则认为检测到障碍
        self.laser_from_camera_update_ranges = None # 存储相机坐标系下的点在odom坐标系下的坐标
        self.detect_flag = False
        self.last_time = 0;

        self.lidar_listener = rospy.Subscriber('scan', LaserScan, self.lidar_callback)
        self.camera_lisener = rospy.Subscriber('camera_scan', LaserScan, self.camera_callback)
        #self.camera_lisener.unregister()
        #self.camera_lisener = rospy.Subscriber('camera_scan', LaserScan, self.camera_callback)
        
        self.laser_publisher = rospy.Publisher('laser_fusion', LaserScan, queue_size = 1)

        # 获取相机坐标系下点在雷达坐标系下坐标
        self.transformer = tf.TransformListener()

        rospy.spin()
            

    def camera_callback(self, laser_from_camera):
        if not self.detect_flag:
            self.laser_from_camera = laser_from_camera

    def lidar_callback(self, laser_from_lidar):
        self.laser_from_lidar = laser_from_lidar
        #print "Receive lidar data!"
      
        # 确保两个话题都接收到了数据
        if self.laser_from_camera != None:
            
            # 雷达角增量与相机角增量
            lidar_angle_increment = self.laser_from_lidar.angle_increment
            camera_angle_increment = self.laser_from_camera.angle_increment

            # 相机与雷达角最大最小值
            camera_angle_min = self.laser_from_camera.angle_min
            camera_angle_max = self.laser_from_camera.angle_max
            lidar_angle_min = self.laser_from_lidar.angle_min
            lidar_angle_max = self.laser_from_lidar.angle_max
            
            laser_fusion = list(self.laser_from_lidar.ranges)
           
            # 记录相机数据角度
            angles = []

            for i in range(len(self.laser_from_camera.ranges)):
                angles.append(camera_angle_min + i*camera_angle_increment)

            update_data = 0
            if self.detect_flag:
                
                if (rospy.Time.now().secs - self.last_time.secs < self.duration):
                    #self.laser_from_camera.ranges = list(self.laser_from_camera.ranges)
                    ranges_list = list(self.laser_from_camera.ranges)

                    # odom转换到相机坐标系下
                    for i in range(len(self.laser_from_camera_update_ranges)):
                        curr_range = self.laser_from_camera_update_ranges[i][1]
                        if not (math.isnan(curr_range) or math.isinf(curr_range)):
                            (angles[i], distance_camera) = self.transform_from_source_to_target(self.laser_from_camera_update_ranges[i][0], curr_range, self.odom_frame, self.depth_camera_frame)
                        
                            ranges_list[i] = distance_camera
                        else:
                            ranges_list[i] = float("nan")
                    # 检测到物体用之前的数据替换当前数据
                    self.laser_from_camera.ranges = tuple(ranges_list)
                        
                else:
                    self.detect_flag = False

            # 相机坐标系下数据转换到雷达坐标系下
            for i in range(len(self.laser_from_camera.ranges)):
                curr_range = self.laser_from_camera.ranges[i]
                if not (math.isnan(curr_range) or math.isinf(curr_range)):
                    (angle_l, distance_l) = self.transform_from_source_to_target(angles[i], self.laser_from_camera.ranges[i], self.depth_camera_frame, self.lidar_frame)
                    #(angle_l, distance_l) = self.transform_from_camera_to_lidar(angles[i], self.laser_from_camera.ranges[i])
                    #(angle_l, distance_l) = self.transform_from_camera_to_lidar(camera_angle_min + i*camera_angle_increment, self.laser_from_camera.ranges[i])
                    if math.isnan(angle_l):
                        continue

                    # 获取转换后角度对应的激光雷达数组索引
                    index = (int)((angle_l - lidar_angle_min) / lidar_angle_increment)+1
                    # 判断当前雷达值与相机值取小并更新
                    if laser_fusion[index] > distance_l:
                        if self.debug:
                            rospy.logdebug("index %d is updated from %f to %f",index, laser_fusion[index], distance_l)
                        
                       # print ("index %d is updated from %f to %f",index, laser_fusion[index], distance_l)
                        laser_fusion[index] = distance_l
                        
                        if not self.detect_flag:
                            update_data += 1

            '''
                发布融合数据
            ''' 
            # 设置新数据当前时间
            # self.laser_from_lidar.header.stamp = rospy.Time.now()
            self.laser_from_lidar.ranges = tuple(laser_fusion)
            self.laser_publisher.publish(self.laser_from_lidar)

            # 判断是否检测到障碍
            if update_data >= 30:
                self.detect_flag = True
                self.last_time = rospy.Time.now()
                self.laser_from_camera_update_ranges = []
                for i in range(len(self.laser_from_camera.ranges)):
                    (angle_t, distance_t) = self.transform_from_source_to_target(camera_angle_min + i*camera_angle_increment, self.laser_from_camera.ranges[i], self.depth_camera_frame, self.odom_frame)
                    self.laser_from_camera_update_ranges.append((angle_t, distance_t))
                #self.camera_lisener.unregister()
            
            # 清空两个话题数据重新接收
            #self.laser_from_lidar = None
            #self.laser_from_camera = None
            
    def transform_from_camera_to_lidar(self, angle, distance):
        # 相机极坐标系转化为直角坐标系
        xc = distance * math.cos(angle)
        yc = distance * math.sin(angle)
        
        ps_camera = PointStamped()
        ps_camera.header.stamp = rospy.Time(0) 
        #ps_camera.header.frame_id = "camera_link"
        ps_camera.header.frame_id = self.depth_camera_frame
        ps_camera.point.x = xc 
        ps_camera.point.y = yc 
        ps_camera.point.z = 0 # 投影后不会用到z坐标值，故可以设置为任意值
        
        try:
            #ps_lidar = self.transformer.transformPoint("/base_link", ps_camera)
            ps_lidar = self.transformer.transformPoint(self.lidar_frame, ps_camera)
            xl = ps_lidar.point.x
            yl = ps_lidar.point.y
            
            if self.debug:
                rospy.logdebug('camera:' + str(xc) + '    ' + str(yc))
                rospy.logdebug('laser:' + str(xl) + '    ' + str(yl))
            
        except (tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException):
            rospy.logerr("tf transform failed.")
            raise

        angle_lidar = math.atan2(yl, xl)
        distance_lidar = math.sqrt(xl ** 2 + yl ** 2)

        return (angle_lidar, distance_lidar)

    def transform_from_source_to_target(self, angle, distance, source, target):
            
        # 极坐标系转化为直角坐标系
        xs = distance * math.cos(angle)
        ys = distance * math.sin(angle)
        
        ps_source = PointStamped()
        ps_source.header.stamp = rospy.Time(0) 
        
        ps_source.header.frame_id = source
        ps_source.point.x = xs 
        ps_source.point.y = ys 
        ps_source.point.z = 0 # 投影后不会用到z坐标值，故可以设置为任意值
        
        try:
            #ps_lidar = self.transformer.transformPoint("/base_link", ps_camera)
            ps_target = self.transformer.transformPoint(target, ps_source)
            xt = ps_target.point.x
            yt = ps_target.point.y
            
            if self.debug:
                rospy.logdebug('source:' + str(xs) + '    ' + str(ys))
                rospy.logdebug('target:' + str(xt) + '    ' + str(yt))
            #print ('source:' + str(xs) + '    ' + str(ys)+ '    ' + 'angle: ' + str(angle))
            #print ('target:' + str(xt) + '    ' + str(yt)+ '    ' + 'distance: ' + str(distance))
            
        except (tf.ConnectivityException, tf.LookupException, tf.ExtrapolationException):
            rospy.logerr("tf transform failed.")
            raise

        angle_target = math.atan2(yt, xt)
        distance_target = math.sqrt(xt ** 2 + yt ** 2)

        return (angle_target, distance_target)

if __name__ == '__main__':
    try:
        LaserFusion()
    except rospy.ROSInterruptException:
        pass
