/**
 * Filename: MapGraph.h
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

#ifndef INCLUDE_STC_PATROL_MAPGRAPH_H_
#define INCLUDE_STC_PATROL_MAPGRAPH_H_


#include <algorithm>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/typeof/typeof.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>


#define foreach BOOST_FOREACH


using namespace std;


struct Vertex {

    static const Vertex TOP;
    static const Vertex BOTTOM;
    static const Vertex LEFT;
    static const Vertex RIGHT;

    Vertex() : x(0), y(0) {
    }

    Vertex(uint32_t x, uint32_t y)
        : x(x), y(y) {
    }

    Vertex operator+(const Vertex& vertex) const {
        return Vertex(this->x + vertex.x, this->y + vertex.y);
    }

    Vertex operator*(double factor) const {
        return Vertex(this->x * factor, this->y * factor);
    }

    bool operator <(const Vertex& vertex) const {
        return this->encode() < vertex.encode();
    }

    size_t operator()() const {
        return encode();
    }

    bool operator==(const Vertex& vertex) const {
        return this->x == vertex.x &&
                this->y == vertex.y;
    }

    uint64_t encode() const {
        uint64_t encoded = (static_cast<uint64_t>(x) << 32) + y;
        return encoded;
    }

    uint32_t x;
    uint32_t y;

};

struct VertexHasher : std::unary_function<Vertex, size_t> {

    size_t operator()(const Vertex& vertex) const {
        return vertex.encode();
    }

};

struct Edge {

    Edge(const Vertex& vertex1, const Vertex& vertex2)
        : vertex1(vertex1), vertex2(vertex2) {
    }

    bool operator==(const Edge& edge) const {
        return this->vertex1 == edge.vertex1 &&
                this->vertex2 == edge.vertex2;
    }

    Vertex vertex1;
    Vertex vertex2;

};

struct EdgeHasher : std::unary_function<Edge, size_t> {

    size_t operator()(const Edge& edge) const {
        return edge.vertex1() + edge.vertex2();
    }

};


/*
 * Map graph
 */
class MapGraph {

public:

    MapGraph();

public:

    /**
     * Adds an edge to the graph
     * @param vertex1
     * @param vertex2
     */
    void addEdge(const Vertex& vertex1, const Vertex& vertex2);

    /**
     * Gets all neighbors of provided vertex
     * @param vertex
     * @return
     */
    vector<Vertex> getNeighbors(const Vertex& vertex) const;

    /**
     * Checks if specified edge exists
     * @param vertex1
     * @param vertex2
     * @return
     */
    bool edgeExists(const Vertex& vertex1, const Vertex& vertex2) const;

    /**
     * Checks if specified vertex exists
     * @param vertex
     * @return
     */
    bool vertexExists(const Vertex& vertex) const;

    /**
     * Gets all edges
     * @return
     */
    vector<Edge> getEdges() const;

    /**
     * Gets all vertices
     * @return
     */
    vector<Vertex> getVertices() const;

private:

    boost::unordered_set<Vertex, VertexHasher> vertices_;
    boost::unordered_multimap<Vertex, Vertex, VertexHasher> edges_;

};


#endif /* INCLUDE_STC_PATROL_MAPGRAPH_H_ */
