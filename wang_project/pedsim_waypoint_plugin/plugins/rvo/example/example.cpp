/*
 * RVO Library
 * example.cpp
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

#ifndef _WIN32_WINNT    // Allow use of features specific to Windows XP or later.
#define _WIN32_WINNT 0x0501  // Change this to the appropriate value to target other versions of Windows.
#endif

#include <stdio.h>
#include <stdlib.h>

#ifdef _MSC_VER
#include <tchar.h>
#else
#define _TCHAR char
#define _tmain main
#endif

#include "RVOSimulator.h"

void setupScenario( RVO::RVOSimulator * sim ) {
  // Specify global time step of the simulation
  sim->setTimeStep( 0.25f );

  // Specify default parameters for agents that are subsequently added
  sim->setAgentDefaults( 250, 15.0f, 10, 2.0f, 3.0f, 1.0f, 2.0f, 7.5f, 1.0f );

  // Add agents (and simulataneously their goals), specifying their start position and goal ID
  sim->addAgent( RVO::Vector2(-50.0f, -50.0f), sim->addGoal( RVO::Vector2(50.0f, 50.0f) ) );
  sim->addAgent( RVO::Vector2(50.0f, -50.0f), sim->addGoal( RVO::Vector2(-50.0f, 50.0f) ) );
  sim->addAgent( RVO::Vector2(50.0f, 50.0f), sim->addGoal( RVO::Vector2(-50.0f, -50.0f) ) );
  sim->addAgent( RVO::Vector2(-50.0f, 50.0f), sim->addGoal( RVO::Vector2(50.0f, -50.0f) ) );

  // Add (line segment) obstacles, specifying both endpoints of the line segments
  sim->addObstacle( RVO::Vector2(-7.0f, -20.0f), RVO::Vector2(-7.0f, 20.0f) );
  sim->addObstacle( RVO::Vector2(-7.0f, 20.0f), RVO::Vector2(7.0f, 20.0f) );
  sim->addObstacle( RVO::Vector2(7.0f, 20.0f), RVO::Vector2(7.0f, -20.0f) );
  sim->addObstacle( RVO::Vector2(7.0f, -20.0f), RVO::Vector2(-7.0f, -20.0f) );

  // Add roadmap vertices, specifying their position
  sim->addRoadmapVertex( RVO::Vector2(-10.0f, -23.0f) );
  sim->addRoadmapVertex( RVO::Vector2(-10.0f, 23.0f) );
  sim->addRoadmapVertex( RVO::Vector2(10.0f, 23.0f) );
  sim->addRoadmapVertex( RVO::Vector2(10.0f, -23.0f) );

  // Do not automatically create edges between mutually visible roadmap vertices
  sim->setRoadmapAutomatic( -1 );

  // Manually specify edges between vertices, specifying the ID's of the vertices the edges connect
  sim->addRoadmapEdge( 0, 1 );
  sim->addRoadmapEdge( 1, 2 );
  sim->addRoadmapEdge( 2, 3 );
  sim->addRoadmapEdge( 3, 0 );
}


void updateVisualization( RVO::RVOSimulator * sim ) {
  // Output the current global time
  std::cout << sim->getGlobalTime() << " ";

  // Output the position and orientation for all the agents
  for (int i = 0; i < sim->getNumAgents(); ++i) {
    std::cout << sim->getAgentPosition( i ) << " " << sim->getAgentOrientation( i ) << " ";
  }

  std::cout << std::endl;
}


int _tmain(int argc, _TCHAR* argv[])
{
  // Create a simulator instance
  RVO::RVOSimulator * sim = RVO::RVOSimulator::Instance();

  // Set up the scenario
  setupScenario( sim );

  // Initialize the simulation
  sim->initSimulation();

  // Perform (and manipulate) the simulation
  do {
    updateVisualization( sim );
    sim->doStep();
  } while ( !sim->getReachedGoal() );

  delete sim;

  return 0;
}
