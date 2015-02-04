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

#include <stc_patrol/StcPathPlanner.h>


StcPathPlanner::StcPathPlanner() {
}

nav_msgs::Path::Ptr StcPathPlanner::plan(
        const nav_msgs::OccupancyGrid& map) const {
    nav_msgs::OccupancyGrid::Ptr coarseMap = createCoarseMap(map);
    MapGraph spanningTree = createSpanningTree(*coarseMap);
//    nav_msgs::Path path = extractPathFromSpanningTree(spanningTree);
}

nav_msgs::OccupancyGrid::Ptr StcPathPlanner::createCoarseMap(
        const nav_msgs::OccupancyGrid& map) const {

    int factor = 24;

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
        const nav_msgs::OccupancyGrid& map) const {

    Vertex startVertex;
    bool freeVertexFound = false;
    // Find first free vertex
    for (int y = 0; y < map.info.height; ++y)
        for (int x = 0; x < map.info.width; ++x)
            if (map.data[y * map.info.width + x] == 0) {
                startVertex = Vertex(x, y);
                freeVertexFound = true;
                break;
            }

    if (!freeVertexFound)
        throw new string("Free cell not found on coarse map");

    // BFS
    MapGraph graph;
    deque<Vertex> discovered;

    discovered.push_back(startVertex);

    while (!discovered.empty()) {
        Vertex currentVertex = discovered.front();
        discovered.pop_front();

        cout << currentVertex.x << ", " << currentVertex.y << endl;

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

MapGraph StcPathPlanner::extractPathFromSpanningTree(
        const MapGraph& graph) const {

    MapGraph fineGridGraph;

    double orientation = 0;
    Vertex coarseCellStart = graph.getEdges()[0].vertex1;
    Vertex fineCell(coarseCellStart.x * 2, coarseCellStart.y * 2);

    // Create inter-cell edges
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

    return fineGridGraph;
}
