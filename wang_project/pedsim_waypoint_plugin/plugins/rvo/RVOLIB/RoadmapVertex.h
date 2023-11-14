/*
 * RVO Library
 * RoadmapVertex.h
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO/>
 */

/* \file RoadmapVertex.h Contains the class RoadmapVertex. */

#ifndef __ROADMAP_VERTEX_H__
#define __ROADMAP_VERTEX_H__

#include "RVODef.h"

namespace RVO {

  /* The class defining a roadmap vertex. */
  class RoadmapVertex
  {
  private:
    /* Constructor. Constructs a roadmap vertex.
      \param p The position of the roadmap vertex. */
    RoadmapVertex(const Vector2& p);
    /* Deconstructor. */
    ~RoadmapVertex();

    /* Adds all visible roadmap vertices for the specified radius to the list of neighbors.
    */
    void computeNeighbors(float radius);
    /* Adds a roadmap vertex to the list of neighbors.
      \param distance The distance to the neighboring roadmap vertex (used for shortest path planning).
      \param neighbor_id The ID of the neighboring roadmap vertex. */
    void addNeighbor(float distance, int neighbor_id);

    /* The position of the roadmap vertex */
    Vector2 _p;

    /* The list of neighbors of the roadmap vertex. The vector contains pairs of distance to the neighbor and roadmap vertex ID of the neighbor. */
    std::vector<std::pair<float, int> > _neighbors;  // list of neighbors (distance to neighboring vertex, index of neighbor vertex)

  protected:
    /* A reference to the singleton simulator. */
    static RVOSimulator* _sim;

    friend class Agent;
    friend class Roadmap;
    friend class Goal;
    friend class RVOSimulator;
  };
}

#endif
