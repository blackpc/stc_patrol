/**
 * Filename: MapGraph.cpp
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


#include <coverage/MapGraph.h>

const Vertex Vertex::TOP = Vertex(0, -1);
const Vertex Vertex::BOTTOM = Vertex(0, 1);
const Vertex Vertex::LEFT = Vertex(-1, 0);
const Vertex Vertex::RIGHT = Vertex(1, 0);

MapGraph::MapGraph()
    : originX_(0), originY_(0), resolution_(0) {

}

void MapGraph::addEdge(const Vertex& vertex1, const Vertex& vertex2) {
    this->vertices_.insert(vertex1);
    this->vertices_.insert(vertex2);

    if (!edgeExists(vertex1, vertex2)) {
        this->edges_.insert(make_pair(vertex1, vertex2));
        this->edges_.insert(make_pair(vertex2, vertex1));
    }
}

vector<Vertex> MapGraph::getNeighbors(const Vertex& vertex) const {
    vector<Vertex> neighbors;

    BOOST_AUTO(range, this->edges_.equal_range(vertex));
    for (BOOST_AUTO(it, range.first); it != range.second; ++it) {
        neighbors.push_back(it->second);
    }

    return neighbors;
}

bool MapGraph::edgeExists(const Vertex& vertex1, const Vertex& vertex2) const {
    BOOST_AUTO(range1, this->edges_.equal_range(vertex1));
    for (BOOST_AUTO(it, range1.first); it != range1.second; ++it) {
        if (it->second == vertex2) {
            return true;
        }
    }

    BOOST_AUTO(range2, this->edges_.equal_range(vertex2));
    for (BOOST_AUTO(it, range2.first); it != range2.second; ++it) {
        if (it->second == vertex1) {
            return true;
        }
    }

    return false;
}

bool MapGraph::vertexExists(const Vertex& vertex) const {
    return vertices_.count(vertex) > 0;
}

vector<Edge> MapGraph::getEdges() const {
    vector<Edge> edges;

    foreach(const Vertex& vertex, vertices_) {
        foreach(const Vertex& neighbor, getNeighbors(vertex)) {
            edges.push_back(Edge(vertex, neighbor));
        }
    }

    return edges;
}

vector<Vertex> MapGraph::getVertices() const {
    vector<Vertex> vertices;

    foreach(const Vertex& vertex, vertices_) {
        vertices.push_back(vertex);
    }

    return vertices;
}
