/*
 * RVO Library
 * Obstacle.h
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

/* \file Obstacle.h Contains the class Obstacle. */

#ifndef __OBSTACLE_H__
#define __OBSTACLE_H__

#include "RVODef.h"

namespace RVO {

  /* The class defining a line segment obstacle. */
  class Obstacle
  {
  private:
    /* Constructor. Constructs an obstacle.
      \param a The first endpoint of the obstacle.
      \param b The second endpoint of the obstacle. */
    Obstacle(const Vector2& a, const Vector2& b);
    /* Deconstructor. */
    ~Obstacle();

    /* The first endpoint of the obstacle. */
    Vector2 _p1;
    /* The second endpoint of the obstacle. */
    Vector2 _p2;

    /* The normal vector of the line segment obstacle. */
    Vector2 _normal;

    friend class Agent;
    friend class RoadmapVertex;
    friend class KDTree;
    friend class RVOSimulator;
  };
}    // RVO namespace

#endif
