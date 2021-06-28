#!/usr/bin/env python3
from __future__ import print_function

import math
import time

import rospy
import tf2_ros
from enc_query.query import Query
from geographic_msgs.msg import GeoPose, GeoPath, GeoPoint
from geographic_visualization_msgs.msg import GeoVizItem, GeoVizPolygon, GeoVizPointList
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from tf.transformations import euler_from_quaternion
from tf2_geometry_msgs import do_transform_pose

import project11


def calculate_fov(heading, position, distance):
    if heading is not None and position is not None:
        long = position[1]
        lat = position[0]
        angle = heading
        points = []
        for i in range(9):
            point_radians = project11.geodesic.direct(long, lat, angle, distance)
            point_degrees = (math.degrees(point_radians[0]), math.degrees(point_radians[1]))
            points.append(point_degrees)
            angle = angle + math.radians(45)
        # rospy.logerr(points)
        return points


def points_to_path(points):
    # init path and add points
    # rospy.logerr(points)
    fov_path = GeoPath()

    for i in range(len(points)):
        pose = GeoPose()
        pose.position.longitude = points[i][0]
        pose.position.latitude = points[i][1]
        fov_path.poses.append(pose)
    rospy.logerr(fov_path)
    return fov_path


def geopath_to_geovizpoly(path):
    poly = GeoVizPolygon()
    for geoPose in path.poses:
        poly.outer.points.append(geoPose.position)
    poly.fill_color = ColorRGBA(1, 0, 0, .2)
    return poly


class FOVQuery:
    def __init__(self, distance):
        self.distance = distance
        self.time_log = [0, 0]
        self.odometry = None
        rospy.Subscriber('odom', Odometry, self.odometry_callback, queue_size=1)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # init VizItem and add polygon
        self.fov_viz_item = GeoVizItem()
        self.fov_viz_item.polygons.append(GeoPath())
        self.fov_viz_item.id = "FOV"

        self.query = Query("/home/thomasdonnelly/Downloads/ENC_ROOT")

    def iterate(self, data):
        # calculate fov for current position
        fov_points = calculate_fov(self.heading(), self.position(), self.distance)
        if fov_points is not None:
            fov_path = points_to_path(fov_points)

            # update VizItem
            self.fov_viz_item.polygons[0] = geopath_to_geovizpoly(fov_path)
            # rospy.logerr( self.fov_viz_item.polygons[0])
            t = time.time()
            features = self.query.query(fov_points)
            # features = self.query.query([(-70.855, 43.123), (-70.855, 43.12), (-70.863, 43.12), (-70.863, 43.123)])
            t = time.time() - t
            self.time_log[0] += t
            self.time_log[1] += 1
            rospy.loginfo("Query Time:" + str(t))
            rospy.loginfo("Time Log:" + str(self.time_log))

            # draw points in view
            feature_list = GeoVizPointList()
            feature_list.color = ColorRGBA(0, 1, 0, 1)
            feature_list.size = 20
            feature_viz = GeoVizItem()
            feature_viz.point_groups = [None]

            for feature in features:
                long = feature[1]
                lat = feature[2]
                # rospy.logerr(str(long) +" "+ str(lat))

                feat = GeoPoint(lat, long, 0)
                feature_list.points.append(feat)

            feature_viz.point_groups[0] = feature_list

            # rospy.logerr("publishing")
            # #pubish updates
            geoVizItem_pub.publish(self.fov_viz_item)
            geoVizItem_pub.publish(feature_viz)

    def odometry_callback(self, msg):
        self.odometry = msg

    def position(self):
        if self.odometry is not None:
            try:
                odom_to_earth = self.tfBuffer.lookup_transform("earth", self.odometry.header.frame_id, rospy.Time())
            except Exception as e:
                print(e)
                return
            ecef = do_transform_pose(self.odometry.pose, odom_to_earth).pose.position
            return project11.wgs84.fromECEFtoLatLong(ecef.x, ecef.y, ecef.z)

    def heading(self):
        if self.odometry is not None:
            o = self.odometry.pose.pose.orientation
            q = (o.x, o.y, o.z, o.w)
            return math.radians(90) - euler_from_quaternion(q)[2]


if __name__ == "__main__":
    rospy.init_node('enc_query')

    geoVizItem_pub = rospy.Publisher('project11/display', GeoVizItem, queue_size=10)

    fov_query = FOVQuery(2000)
    rospy.Timer(rospy.Duration.from_sec(.1), fov_query.iterate)

    rospy.spin()
