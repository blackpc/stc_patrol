/**
 * Filename: StcPathPlanner.h
 *   Author: Igor Makhtes
 *     Date: Feb 4, 2015
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

#ifndef INCLUDE_STC_PATROL_STCPATHPLANNER_H_
#define INCLUDE_STC_PATROL_STCPATHPLANNER_H_


#include <deque>
#include <algorithm>

#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include <coverage/MapGraph.h>


#define foreach BOOST_FOREACH


using namespace std;


/*
 * Spanning tree coverage path planner
 */
class StcPathPlanner {

public:

    StcPathPlanner(double coverageWidth);

public:

    /**
     * Extracts coverage plan from provided map
     * @param map
     * @param initialPosition Initial world position of the robot, relative to 'map' frame
     *                        used to determine the initial cell on fine and coarse grids
     * @return
     */
    nav_msgs::Path::Ptr plan(const nav_msgs::OccupancyGrid& map,
            const tf::Vector3& initialPosition) const;

private:

    /**
     * The width of the coverage tool
     * the coarse grid cell size will be twice this width
     */
    double coverageWidth_;

    /**
     * Coarse map publisher
     */
    ros::Publisher mapPublisher_;

    /**
     * Spanning tree visual markers publisher
     */
    ros::Publisher visPublisher_;

    /**
     * Path publisher
     */
    ros::Publisher pathPublisher_;

private:

    /**
     * Converts map coordinates to a grid cell vertex
     * @param coordinates
     * @return
     */
    Vertex mapToVertex(tf::Vector3 coordinates, const MapGraph& graph) const;

    /**
     * Creates coarse map from provided fine map
     * @param map
     * @return
     */
    nav_msgs::OccupancyGrid::Ptr createCoarseMap(const nav_msgs::OccupancyGrid& map) const;

    /**
     * Creates spanning tree graph out of provided map
     * @param map
     * @return
     */
    MapGraph createSpanningTree(const nav_msgs::OccupancyGrid& map,
            const tf::Vector3& initialPosition) const;

    /**
     * Extracts a path around spanning tree
     * @param graph
     * @return
     */
    nav_msgs::Path::Ptr extractPathFromSpanningTree(const MapGraph& graph,
            const tf::Vector3& initialPosition) const;

    /**
     * Publishes spanning tree
     * @param graph
     */
    void publishSpanningTree(const MapGraph& graph) const;

};


#endif /* INCLUDE_STC_PATROL_STCPATHPLANNER_H_ */
