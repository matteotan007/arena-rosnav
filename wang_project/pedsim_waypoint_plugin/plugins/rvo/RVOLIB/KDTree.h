/*
 * RVO Library
 * KDTree.h
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

#ifndef __KDTREE_H__
#define __KDTREE_H__

#define MAX_LEAF_SIZE 10

#include "RVODef.h"

namespace RVO {
  class KDTree
  {

  struct AgentTreeNode {
    int begin;
    int end;
    float minX, maxX, minY, maxY;
    int left;
    int right;
  };

  struct ObstacleTreeNode {
    int obstacleID; // is negative when trivial empty node
    ObstacleTreeNode* left;
    ObstacleTreeNode* right;
  };

  private:
    KDTree(void);
    ~KDTree(void);

    std::vector<AgentTreeNode> _agentTree;
    std::vector<int> _agentIDs;
    ObstacleTreeNode* _obstacleTree;

    void buildAgentTreeRecursive(int begin, int end, int node);
    inline void buildAgentTree() { if (!_agentIDs.empty()) buildAgentTreeRecursive(0, (int) _agentIDs.size(), 0); }
    void queryAgentTreeRecursive(Agent* agent, float& rangeSq, int node) const;
    inline void computeAgentNeighbors(Agent* agent, float& rangeSq) const { queryAgentTreeRecursive(agent, rangeSq, 0); }

    void deleteObstacleTree(ObstacleTreeNode* node);
    ObstacleTreeNode* buildObstacleTreeRecursive(const std::vector<int>& obstacles);
    void buildObstacleTree();
    void queryObstacleTreeRecursive(Agent* agent, float& rangeSq, ObstacleTreeNode* node) const;
    inline void computeObstacleNeighbors(Agent* agent, float& rangeSq) const { queryObstacleTreeRecursive(agent, rangeSq, _obstacleTree); }

    bool queryVisibilityRecursive(const Vector2& q1, const Vector2& q2, float radius, ObstacleTreeNode* node) const;
    inline bool queryVisibility(const Vector2& q1, const Vector2& q2, float radius) const { return queryVisibilityRecursive(q1, q2, radius, _obstacleTree); }

  protected:
    /* A reference to the singleton simulator. */
    static RVOSimulator*  _sim;

    friend class Agent;
    friend class RoadmapVertex;
    friend class RVOSimulator;
  };
}

#endif
