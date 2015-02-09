/**
 * Filename: StcPathPlanner.cpp
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

#include <coverage/StcPathPlanner.h>


StcPathPlanner::StcPathPlanner(double coverageWidth)
    : coverageWidth_(coverageWidth) {

    ros::NodeHandle nodePrivate("~");

    mapPublisher_ = nodePrivate.advertise<nav_msgs::OccupancyGrid>("/map_coarse", true, 1);
    visPublisher_ = nodePrivate.advertise<visualization_msgs::MarkerArray>("/spanning_tree", true, 1);
}

nav_msgs::Path::Ptr StcPathPlanner::plan(
        const nav_msgs::OccupancyGrid& map, const tf::Vector3& initialPosition) const {
    ROS_INFO("Creating coarse map...");
    nav_msgs::OccupancyGrid::Ptr coarseMap = createCoarseMap(map);
    mapPublisher_.publish(coarseMap);

    ROS_INFO("Creating spanning tree...");
    MapGraph coarseGraph = createSpanningTree(*coarseMap, initialPosition);
    publishSpanningTree(coarseGraph);

    ROS_INFO("Extracting path...");
    nav_msgs::Path::Ptr path = extractPathFromSpanningTree(coarseGraph, initialPosition);

    return path;
}

nav_msgs::OccupancyGrid::Ptr StcPathPlanner::createCoarseMap(
        const nav_msgs::OccupancyGrid& map) const {

    /**
     * Multiplication factor of the original map relative to coarse map
     */
    int factor = 2 * (int)round(coverageWidth_ / map.info.resolution);

    nav_msgs::OccupancyGrid::Ptr coarseMap(new nav_msgs::OccupancyGrid());
    coarseMap->header = map.header;
    coarseMap->info = map.info;
    coarseMap->info.resolution *= factor;
    coarseMap->info.height /= factor;
    coarseMap->info.width  /= factor;
    coarseMap->data.resize(coarseMap->info.width * coarseMap->info.height);

    for (int y = 0; y < coarseMap->info.height; ++y)
        for (int x = 0; x < coarseMap->info.width; ++x) {

            int maxValue = -1;
            for (int coarseY = 0; coarseY < factor; ++coarseY) {
                for (int coarseX = 0; coarseX < factor; ++coarseX) {
                    int cellValue = map.data[(y * factor + coarseY) * map.info.width + x * factor + coarseX];
                    if (cellValue > maxValue)
                        maxValue = cellValue;
                }
            }
            coarseMap->data[y * coarseMap->info.width + x] = maxValue;
        }

    return coarseMap;
}

MapGraph StcPathPlanner::createSpanningTree(
        const nav_msgs::OccupancyGrid& map,
        const tf::Vector3& initialPosition) const {

    // BFS
    // No need for kruskal or prim because we don't have weights,
    // so BFS or DFS will do the work
    MapGraph graph;

    graph.setResolution(map.info.resolution);
    graph.setOriginX(map.info.origin.position.x);
    graph.setOriginY(map.info.origin.position.y);

    Vertex startVertex = mapToVertex(initialPosition, graph);

    deque<Vertex> discovered;

    discovered.push_back(startVertex);

    while (!discovered.empty()) {
        Vertex currentVertex = discovered.front();
        discovered.pop_front();

        Vertex neighbor;

        // Left
        neighbor = Vertex(currentVertex.x - 1, currentVertex.y);
        if (currentVertex.x > 0 &&
                map.data[currentVertex.y * map.info.width + currentVertex.x - 1] == 0 &&
                !graph.vertexExists(neighbor)) {
            discovered.push_back(neighbor);
            graph.addEdge(currentVertex, neighbor);
        }

        // Right
        neighbor = Vertex(currentVertex.x + 1, currentVertex.y);
        if (currentVertex.x < map.info.width - 1 &&
                map.data[currentVertex.y * map.info.width + currentVertex.x + 1] == 0 &&
                !graph.vertexExists(neighbor)) {
            discovered.push_back(neighbor);
            graph.addEdge(currentVertex, neighbor);
        }

        // Top
        neighbor = Vertex(currentVertex.x, currentVertex.y - 1);
        if (currentVertex.y > 0 &&
                map.data[(currentVertex.y - 1) * map.info.width + currentVertex.x] == 0 &&
                !graph.vertexExists(neighbor)) {
            discovered.push_back(neighbor);
            graph.addEdge(currentVertex, neighbor);
        }

        // Bottom
        neighbor = Vertex(currentVertex.x, currentVertex.y + 1);
        if (currentVertex.y < map.info.height - 1 &&
                map.data[(currentVertex.y + 1) * map.info.width + currentVertex.x] == 0 &&
                !graph.vertexExists(neighbor)) {
            discovered.push_back(neighbor);
            graph.addEdge(currentVertex, neighbor);
        }
    }

    return graph;
}

nav_msgs::Path::Ptr StcPathPlanner::extractPathFromSpanningTree(
        const MapGraph& graph, const tf::Vector3& initialPosition) const {

    MapGraph fineGridGraph;
    fineGridGraph.setResolution(graph.getResolution() / 2.0);
    fineGridGraph.setOriginX(graph.getOriginX());
    fineGridGraph.setOriginY(graph.getOriginY());

    Vertex coarseCellStart = graph.getEdges()[0].vertex1;
    Vertex fineCell(coarseCellStart.x * 2, coarseCellStart.y * 2);

    // Create fine graph - the path graph around spanning tree graph
    foreach(const Vertex& cell, graph.getVertices()) {
        if (!graph.edgeExists(cell, cell + Vertex::TOP))
            fineGridGraph.addEdge(cell * 2, cell * 2 + Vertex::RIGHT);
        else {
            fineGridGraph.addEdge(cell * 2, cell * 2 + Vertex::TOP);
            fineGridGraph.addEdge(cell * 2 + Vertex::RIGHT, cell * 2 + Vertex::RIGHT + Vertex::TOP);
        }

        if (!graph.edgeExists(cell, cell + Vertex::BOTTOM))
            fineGridGraph.addEdge(cell * 2 + Vertex::BOTTOM, cell * 2 + Vertex::BOTTOM + Vertex::RIGHT);
        else {
            fineGridGraph.addEdge(cell * 2 + Vertex::BOTTOM, cell * 2  + Vertex::BOTTOM + Vertex::BOTTOM);
            fineGridGraph.addEdge(cell * 2 + Vertex::BOTTOM + Vertex::RIGHT, cell * 2  + Vertex::RIGHT + Vertex::BOTTOM + Vertex::BOTTOM);
        }

        if (!graph.edgeExists(cell, cell + Vertex::LEFT))
            fineGridGraph.addEdge(cell * 2, cell * 2 + Vertex::BOTTOM);
        else {
            fineGridGraph.addEdge(cell * 2, cell * 2 + Vertex::LEFT);
            fineGridGraph.addEdge(cell * 2 + Vertex::BOTTOM, cell * 2 + Vertex::BOTTOM + Vertex::LEFT);
        }

        if (!graph.edgeExists(cell, cell + Vertex::RIGHT))
            fineGridGraph.addEdge(cell * 2 + Vertex::RIGHT, cell * 2 + Vertex::BOTTOM + Vertex::RIGHT );
        else {
            fineGridGraph.addEdge(cell * 2 + Vertex::RIGHT, cell * 2  + Vertex::RIGHT + Vertex::RIGHT);
            fineGridGraph.addEdge(cell * 2 + Vertex::RIGHT + Vertex::BOTTOM, cell * 2 + Vertex::BOTTOM + Vertex::RIGHT + Vertex::RIGHT);
        }
    }


    // The fine graph contains a path around spanning tree.
    // The following algorithm is a simplified version of DFS
    // that assumes each edge has exactly to vertices
    nav_msgs::Path::Ptr path(new nav_msgs::Path());
    path->header.frame_id = "map";
    path->header.stamp = ros::Time::now();

    Vertex currentVertex = mapToVertex(initialPosition, fineGridGraph);
    Vertex prevVertex = currentVertex;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.orientation.w = 1;

    size_t verticesCount = fineGridGraph.getVertices().size();
    for (size_t i = 0; i < verticesCount; ++i) {
        vector<Vertex> neighbors = fineGridGraph.getNeighbors(currentVertex);
        if (neighbors[0] != prevVertex) {
            prevVertex = currentVertex;
            currentVertex = neighbors[0];
        } else {
            prevVertex = currentVertex;
            currentVertex = neighbors[1];
        }


        pose.pose.position.x = prevVertex.x * (graph.getResolution() / 2.0) +
                graph.getOriginX() + graph.getResolution() / 4.0;

        pose.pose.position.y = prevVertex.y * (graph.getResolution() / 2.0) +
                graph.getOriginY() + graph.getResolution() / 4.0;

        path->poses.push_back(pose);

    }

    return path;
}

Vertex StcPathPlanner::mapToVertex(tf::Vector3 coordinates,
        const MapGraph& graph) const {
    Vertex vertex;

    vertex.x = (coordinates.getX() - graph.getOriginX()) / graph.getResolution();
    vertex.y = (coordinates.getY() - graph.getOriginY()) / graph.getResolution();

    return vertex;
}

void StcPathPlanner::publishSpanningTree(const MapGraph& graph) const {
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
    marker.scale.x = 0.1;
    marker.color.a = 0.25;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    foreach(const Edge& edge, graph.getEdges()) {
        geometry_msgs::Point p1;

        p1.x = edge.vertex1.x * (graph.getResolution() / 1.0) +
                graph.getOriginX() + graph.getResolution() / 2.0;

        p1.y = edge.vertex1.y * (graph.getResolution() / 1.0) +
                graph.getOriginY() + graph.getResolution() / 2.0;

        geometry_msgs::Point p2;

        p2.x = edge.vertex2.x * (graph.getResolution() / 1.0) +
                graph.getOriginX() + graph.getResolution() / 2.0;

        p2.y = edge.vertex2.y * (graph.getResolution() / 1.0) +
                graph.getOriginY() + graph.getResolution() / 2.0;

        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }
    arr.markers.push_back(marker);
    visPublisher_.publish(arr);
}
