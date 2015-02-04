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
#include <visualization_msgs/MarkerArray.h>

#include <stc_patrol/StcPathPlanner.h>


using namespace std;


ros::Publisher mapPublisher_;
ros::Publisher visPublisher_;


void mapCallback(const nav_msgs::OccupancyGrid::Ptr& map) {
    StcPathPlanner planner;
    nav_msgs::OccupancyGrid::Ptr coarseMap = planner.createCoarseMap(*map);
    MapGraph graph = planner.createSpanningTree(*coarseMap);
    MapGraph path = planner.extractPathFromSpanningTree(graph);

    mapPublisher_.publish(coarseMap);



    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.025;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    foreach(const Edge& edge, path.getEdges()) {
        geometry_msgs::Point p1;
        p1.x = edge.vertex1.x * (coarseMap->info.resolution / 2.0) + coarseMap->info.origin.position.x + coarseMap->info.resolution / 4.0;
        p1.y = edge.vertex1.y * (coarseMap->info.resolution / 2.0) + coarseMap->info.origin.position.y + coarseMap->info.resolution / 4.0;

        geometry_msgs::Point p2;
        p2.x = edge.vertex2.x * (coarseMap->info.resolution / 2.0) + coarseMap->info.origin.position.x + coarseMap->info.resolution / 4.0;
        p2.y = edge.vertex2.y * (coarseMap->info.resolution / 2.0) + coarseMap->info.origin.position.y + coarseMap->info.resolution / 4.0;

        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }

    arr.markers.push_back(marker);
    visPublisher_.publish(arr);


    cout << "Map published" << endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "stc_patrol_node");
    ros::NodeHandle nodePrivate("~");
    mapPublisher_ = nodePrivate.advertise<nav_msgs::OccupancyGrid>("/map_coarse", true, 1);
    visPublisher_ = nodePrivate.advertise<visualization_msgs::MarkerArray>("/spanning_tree", true, 1);
    ros::Subscriber mapSubscriber = nodePrivate.subscribe("/map", 1, mapCallback);
    ros::spin();
    return 0;
}

