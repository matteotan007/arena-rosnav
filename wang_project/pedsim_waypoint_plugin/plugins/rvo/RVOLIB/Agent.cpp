/*
 * RVO Library
 * Agent.cpp
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
#include "Obstacle.h"
#include "Agent.h"
#include "KDTree.h"

namespace RVO {
  //-----------------------------------------------------------//
  //           Implementation for class: Agent                 //
  //-----------------------------------------------------------//

  RVOSimulator*  Agent::_sim = RVOSimulator::Instance();

  //-----------------------------------------------------------

  Agent::Agent() { }

  //-----------------------------------------------------------

  Agent::Agent( const Vector2& p, int goalID ) {
    _atGoal = false;
    _subGoal = -2;

    _p = p;
    _goalID = goalID;

    _velSampleCount = _sim->_defaultAgent->_velSampleCount;
    _neighborDist = _sim->_defaultAgent->_neighborDist;
    _maxNeighbors = _sim->_defaultAgent->_maxNeighbors;

    _class = _sim->_defaultAgent->_class;
    _r = _sim->_defaultAgent->_r;
    _gR = _sim->_defaultAgent->_gR;
    _prefSpeed = _sim->_defaultAgent->_prefSpeed;
    _maxSpeed = _sim->_defaultAgent->_maxSpeed;
    _maxAccel = _sim->_defaultAgent->_maxAccel;
    _o = _sim->_defaultAgent->_o;
    _safetyFactor = _sim->_defaultAgent->_safetyFactor;
    _v = _sim->_defaultAgent->_v;

  }

  Agent::Agent( const Vector2& p, int goalID, int velSampleCount, float neighborDist, int maxNeighbors, int classID, float r, const Vector2& v, float maxAccel, float gR, float prefSpeed, float maxSpeed, float o, float safetyFactor ) {
    _atGoal = false;
    _subGoal = -2;

    _p = p;
    _goalID = goalID;

    _velSampleCount = velSampleCount;
    _neighborDist = neighborDist;
    _maxNeighbors = maxNeighbors;

    _class = classID;
    _r = r;
    _gR = gR;
    _prefSpeed = prefSpeed;
    _maxSpeed = maxSpeed;
    _maxAccel = maxAccel;
    _o = o;
    _safetyFactor = safetyFactor;
    _v = v;

  }

  //-----------------------------------------------------------

  Agent::~Agent() {  }

  //-----------------------------------------------------------

  // Searching for the best new velocity
  void Agent::computeNewVelocity() {
    float min_penalty = RVO_INFTY;
    Vector2 vCand;

    // Select num_samples candidate velocities within the circle of radius _maxSpeed
    for (int n = 0; n < _velSampleCount; ++n) {
      if (n == 0) {
        vCand = _vPref;
      } else {
        do {
          vCand = Vector2( 2.0f*rand() - RAND_MAX, 2.0f*rand() - RAND_MAX);
        } while (absSq(vCand) > sqr((float) RAND_MAX));
        vCand *= (_maxSpeed / RAND_MAX);
      }

      float dV; // distance between candidate velocity and preferred velocity
      if (_collision) {
        dV = 0;
      } else {
        dV = abs(vCand - _vPref);
      }

      // searching for smallest time to collision
      float ct = RVO_INFTY; // time to collision
      // iterate over neighbors
      for (std::multimap<float, std::pair<int, int> >::iterator j = _neighbors.begin(); j != _neighbors.end(); ++j) {
        float ct_j; // time to collision with agent j
        Vector2 Vab;
        int type = j->second.first;
        int id = j->second.second;

        if (type == AGENT) {
          Agent* other = _sim->_agents[id];
          Vab = 2*vCand - _v - other->_v;
          float time = timeToCollision(_p, Vab, other->_p, _r + other->_r, _collision);
          if (_collision) {
            ct_j = -std::ceil(time / _sim->_timeStep );
            ct_j -= absSq(vCand) / sqr(_maxSpeed);
          } else {
            ct_j = time;
          }
        } else if (type == OBSTACLE) {
          Obstacle* other;
          other = _sim->_obstacles[id];

          float time_1, time_2, time_a, time_b;
          time_1 = timeToCollision(_p, vCand, other->_p1, _r, _collision);
          time_2 = timeToCollision(_p, vCand, other->_p2, _r, _collision);
          time_a = timeToCollision(_p, vCand, other->_p1 + _r * other->_normal, other->_p2 + _r * other->_normal, _collision);
          time_b = timeToCollision(_p, vCand, other->_p1 - _r * other->_normal, other->_p2 - _r * other->_normal, _collision);

          if (_collision) {
            float time = std::max(std::max(std::max(time_1, time_2), time_a), time_b);
            ct_j = -std::ceil(time / _sim->_timeStep);
            ct_j -= absSq(vCand) / sqr(_maxSpeed);
          } else {
            float time = std::min(std::min(std::min(time_1, time_2), time_a), time_b);
            if (time < _sim->_timeStep || sqr(time) < absSq(vCand) / sqr(_maxAccel)) {
              ct_j = time;
            } else {
              ct_j = RVO_INFTY; // no penalty
            }
          }
        }

        if (ct_j < ct) {
          ct = ct_j;
          // pruning search if no better penalty can be obtained anymore for this velocity
          if ( _safetyFactor / ct + dV >= min_penalty) {
            break;
          }
        }
      }

      float penalty = _safetyFactor / ct + dV;
      if (penalty < min_penalty) {
        min_penalty = penalty;
        _vNew = vCand;
      }
    }
  }

  //---------------------------------------------------------------
  void Agent::insertObstacleNeighbor(int id, float& rangeSq) {
    Obstacle* obstacle = _sim->_obstacles[id];
    float distSq = distSqPointLineSegment(obstacle->_p1, obstacle->_p2, _p);
    if (distSq < sqr(_r) && distSq < rangeSq) { // COLLISION!
      if (!_collision) {
        _collision = true;
        _neighbors.clear();
        rangeSq = sqr(_r);
      }

      if (_neighbors.size() == _maxNeighbors) {
        _neighbors.erase(--_neighbors.end());
      }
      _neighbors.insert(std::make_pair(distSq, std::make_pair(OBSTACLE, id)));
      if (_neighbors.size() == _maxNeighbors) {
        rangeSq = (--_neighbors.end())->first;
      }
    } else if (!_collision && distSq < rangeSq) {
      if (_neighbors.size() == _maxNeighbors) {
        _neighbors.erase(--_neighbors.end());
      }
      _neighbors.insert(std::make_pair(distSq, std::make_pair(OBSTACLE, id)));
      if (_neighbors.size() == _maxNeighbors) {
        rangeSq = (--_neighbors.end())->first;
      }
    }
  }

  void Agent::insertAgentNeighbor(int id, float& rangeSq) {
    Agent* other = _sim->_agents[id];
    if (this != other) {
      float distSq = absSq(_p - other->_p);
      if (distSq < sqr(_r + other->_r) && distSq < rangeSq) { // COLLISION!
        if (!_collision) {
          _collision = true;
          _neighbors.clear();
        }

        if (_neighbors.size() == _maxNeighbors) {
          _neighbors.erase(--_neighbors.end());
        }
        _neighbors.insert(std::make_pair(distSq, std::make_pair(AGENT, id)));
        if (_neighbors.size() == _maxNeighbors) {
          rangeSq = (--_neighbors.end())->first;
        }
      } else if (!_collision && distSq < rangeSq) {
        if (_neighbors.size() == _maxNeighbors) {
          _neighbors.erase(--_neighbors.end());
        }
        _neighbors.insert(std::make_pair(distSq, std::make_pair(AGENT, id)));
        if (_neighbors.size() == _maxNeighbors) {
          rangeSq = (--_neighbors.end())->first;
        }
      }
    }
  }

  void Agent::computeNeighbors() {
    // Compute new neighbors of agent;
    // sort them according to distance (optimized effect of pruning heuristic in search for new velocities);
    // seperate between colliding and near agents

    _collision = false;
    _neighbors.clear();

    // check obstacle neighbors
    float rangeSq = std::min(sqr(_neighborDist), sqr(std::max(_sim->_timeStep, _maxSpeed / _maxAccel)*_maxSpeed + _r));
    _sim->_kdTree->computeObstacleNeighbors(this, rangeSq);

    if (_collision) {
      return;
    }

     // Check other agents
    if (_neighbors.size() != _maxNeighbors) {
      rangeSq = sqr(_neighborDist);
    }
    _sim->_kdTree->computeAgentNeighbors(this, rangeSq);
  }

  //---------------------------------------------------------------

  // Prepare for next cycle
  void Agent::computePreferredVelocity()   {
    // compute subgoal
    Goal * goal = _sim->_goals[_goalID];

    if (_subGoal == -1) { // sub_goal is goal
      if (!_sim->_kdTree->queryVisibility(goal->_vertex->_p, _p, 0)) {
        _subGoal = -2;
      }
    } else if (_subGoal >= 0) { // sub_goal is roadmap vertex
      if (_sim->_kdTree->queryVisibility(_sim->_roadmapVertices[_subGoal]->_p, _p, 0)) {
        int try_advance_sub_goal = goal->_dist[_subGoal].second; // try to advance sub_goal
        if (try_advance_sub_goal == -1) { // advanced sub_goal is goal
          if (_sim->_kdTree->queryVisibility(goal->_vertex->_p, _p, _r)) {
            _subGoal = -1;
          }
        } else if (_sim->_kdTree->queryVisibility(_sim->_roadmapVertices[try_advance_sub_goal]->_p, _p, _r)) { // advanced sub_goal is roadmap vertex
          _subGoal = try_advance_sub_goal; // set new sub_goal if visible
        }
      } else { // sub_goal is not visible
        _subGoal = -2;
      }
    }

    if (_subGoal == -2) { // sub_goal is not visible, search all vertices
      if (_sim->_kdTree->queryVisibility(goal->_vertex->_p, _p, _r)) {
        _subGoal = -1;
      } else {
        float mindist = RVO_INFTY;
        for (int i = 0; i < (int) goal->_dist.size(); ++i) {
          float distance = goal->_dist[i].first + abs(_p - _sim->_roadmapVertices[i]->_p);
          if (distance < mindist && _sim->_kdTree->queryVisibility(_sim->_roadmapVertices[i]->_p, _p, _r)) {
            mindist = distance;
            _subGoal = i;
          }
        }
      }
    }

    if (_subGoal == -2) { // no vertex is visible, move in direction of goal
      _subGoal = -1;
    }


    // Set preferred velocity
    Vector2 sub_g;
    if (_subGoal == -1) {
      sub_g = goal->_vertex->_p;
    } else {
      sub_g = _sim->_roadmapVertices[_subGoal]->_p;
    }

    float distSq2subgoal = absSq(sub_g - _p);

    // compute preferred velocity
    if (_subGoal == -1 && sqr(_prefSpeed * _sim->_timeStep) > distSq2subgoal) {
      _vPref = (sub_g - _p) / _sim->_timeStep;
    } else {
      _vPref = _prefSpeed * (sub_g - _p) / std::sqrt(distSq2subgoal);
    }
  }

  //---------------------------------------------------------------

  // update velocity and position of agent
  void Agent::update() {
    // Scale proposed new velocity to obey maximum acceleration
    float dv = abs(_vNew - _v);
    if (dv < _maxAccel * _sim->_timeStep) {
      _v = _vNew;
    } else {
      _v = (1 - (_maxAccel * _sim->_timeStep / dv)) * _v + (_maxAccel * _sim->_timeStep / dv) * _vNew;
    }

    // Update position
    _p += _v * _sim->_timeStep;

    // Set reached goal
    if (absSq(_sim->_goals[_goalID]->_vertex->_p - _p) < sqr(_gR)) {
      _atGoal = true;
    } else {
      _atGoal = false;
      _sim->_allAtGoals = false;
    }

    // Update orientation
    if (!_atGoal) {
      _o = atan(_vPref);
    }
  }

}    // RVO namespace
