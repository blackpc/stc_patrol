/**
 * Filename: stc_patrol_node.cpp
 *   Author: Igor Makhtes
 *     Date: Feb 3, 2015
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>

#include <coverage/StcPathPlanner.h>


using namespace std;


ros::Publisher pathPublisher_;
ros::ServiceClient mapClient_;
tf::TransformListener* tfListener_;

string mapFrame_;
string baseFrame_;
double coverageWidth_;


void beginCoverage(const StcPathPlanner& planner, const nav_msgs::OccupancyGrid& map) {

    tf::StampedTransform robotPoseTf;
    ros::Time poseTime = map.header.stamp;

    ROS_INFO("Looking for robot position...");

    tfListener_->waitForTransform(mapFrame_, baseFrame_, poseTime, ros::Duration(0.1));

    try {
        tfListener_->lookupTransform(mapFrame_, baseFrame_, poseTime, robotPoseTf);
    } catch (tf::TransformException& e) {
        ROS_ERROR_STREAM("Failed to find robot's tf" << endl << "Error message: " << e.what());
        exit(-1);
    }

    ROS_INFO("Robot position received");

    nav_msgs::Path::Ptr path = planner.plan(map, robotPoseTf.getOrigin());
    pathPublisher_.publish(path);
    ROS_INFO("Path published");

    /**
     * TODO implement Waypoint Driver
     */

}

void readParameters(ros::NodeHandle& node) {
    node.param("map_frame", mapFrame_, string("map"));
    node.param("base_frame", baseFrame_, string("base_link"));
    node.param("coverage_width", coverageWidth_, 0.5);
}

nav_msgs::OccupancyGrid requestMap() {
    mapClient_.waitForExistence(ros::Duration(60));

    nav_msgs::GetMap mapService;
    if (mapClient_.call(mapService)) {
        return mapService.response.map;
    }
    else {
        ROS_ERROR("Failed to call service '/static_map'");
        exit(-2);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "coverage_node");
    ros::NodeHandle nodePrivate("~");

    tfListener_ = new tf::TransformListener();

    readParameters(nodePrivate);

    StcPathPlanner coveragePlanner(coverageWidth_);

    pathPublisher_ = nodePrivate.advertise<nav_msgs::Path>("/path", true, 1);
    mapClient_ = nodePrivate.serviceClient<nav_msgs::GetMap>("/static_map");

    boost::this_thread::sleep(boost::posix_time::seconds(1));

    nav_msgs::OccupancyGrid map = requestMap();
    beginCoverage(coveragePlanner, map);

    return 0;
}


