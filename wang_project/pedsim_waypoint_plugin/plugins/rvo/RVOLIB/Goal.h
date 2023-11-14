/*
 * RVO Library
 * Goal.h
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

#ifndef __GOAL_H__
#define __GOAL_H__

#include "RVODef.h"

namespace RVO {
  class Goal
  {
  private:
    Goal(const Vector2& p);
    ~Goal();

    /* Computes the distance from every vertex in the roadmap to the end goal */
    void computeShortestPathTree();

    /* The goal vertex of the agent. Used for computing the shortest path tree. */
    RoadmapVertex* _vertex;
    /* A list of roadmap vertex distances to this goal. The float stores the distance. The int stores the ID of the parent vertex of the roadmap vertex in the shortest path tree (which is -1 if it is the goal). */
    std::vector<std::pair<float, int> > _dist;

  protected:
    /* A reference to the singleton simulator. */
    static RVOSimulator*  _sim;

    friend class Agent;
    friend class RVOSimulator;
  };
}

#endif
