/*
 * RVO Library
 * RoadmapVertex.cpp
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
#include "Obstacle.h"
#include "KDTree.h"

namespace RVO {
  RVOSimulator* RoadmapVertex::_sim = RVOSimulator::Instance();

  RoadmapVertex::RoadmapVertex(const Vector2& p)
  {
    _p = p;
  }

  RoadmapVertex::~RoadmapVertex()
  {
  }

  void RoadmapVertex::computeNeighbors(float radius)
  {
    _neighbors.clear();
    for (int i = 0; i < (int) _sim->_roadmapVertices.size(); ++i) {
      if (_sim->_roadmapVertices[i] != this && _sim->_kdTree->queryVisibility(_p, _sim->_roadmapVertices[i]->_p, radius)) {
        addNeighbor(abs(_sim->_roadmapVertices[i]->_p - _p), i);
      }
    }
  }

  void RoadmapVertex::addNeighbor(float distance, int neighbor_id) {
    _neighbors.push_back(std::make_pair(distance, neighbor_id));
  }
}
