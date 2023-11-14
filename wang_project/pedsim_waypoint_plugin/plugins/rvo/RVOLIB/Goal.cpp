/*
 * RVO Library
 * Goal.cpp
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

#include "RVOSimulator.h"
#include "RoadmapVertex.h"
#include "Goal.h"

namespace RVO {
  RVOSimulator*  Goal::_sim = RVOSimulator::Instance();

  Goal::Goal(const Vector2& p)
  {
    _vertex = new RoadmapVertex(p);
  }

  Goal::~Goal(void)
  {
    delete _vertex;
  }

  //-----------------------------------------------------------

  void Goal::computeShortestPathTree() {
    _vertex->computeNeighbors(_sim->_automaticRadius);

    std::multimap<float, int> Q;
    _dist.assign(_sim->_roadmapVertices.size(), std::make_pair(RVO_INFTY, -1));
    std::vector<std::multimap<float, int>::iterator> pos_in_Q(_sim->_roadmapVertices.size(), Q.end());

    for (int j = 0; j < (int) _vertex->_neighbors.size(); ++j) {
      int u = _vertex->_neighbors[j].second;
      float distance = _vertex->_neighbors[j].first;
      _dist[u] = std::make_pair(distance, -1);
      pos_in_Q[u] = Q.insert(std::make_pair(distance, u));
    }

    int u, v;
    while (!Q.empty()) {
      u = Q.begin()->second;
      Q.erase(Q.begin());
      pos_in_Q[u] = Q.end();

      for (int j = 0; j < (int) _sim->_roadmapVertices[u]->_neighbors.size(); ++j) {
        v = _sim->_roadmapVertices[u]->_neighbors[j].second;
        float dist_uv = _sim->_roadmapVertices[u]->_neighbors[j].first;
        if (_dist[v].first > _dist[u].first + dist_uv) {
          _dist[v] = std::make_pair(_dist[u].first + dist_uv, u);
          if (pos_in_Q[v] == Q.end()) {
            pos_in_Q[v] = Q.insert(std::make_pair(_dist[v].first, v));
          } else {
            Q.erase(pos_in_Q[v]);
            pos_in_Q[v] = Q.insert(std::make_pair(_dist[v].first, v));
          }
        }
      }
    }

  }
}
