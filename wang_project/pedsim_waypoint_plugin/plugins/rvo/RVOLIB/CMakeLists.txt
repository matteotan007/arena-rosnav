#
# RVO Library
# RVOLIB/CMakeLists.txt
#
# Copyright 2008 University of North Carolina at Chapel Hill
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Please send all bug reports to <geom@cs.unc.edu>.
#
# The authors may be contacted via:
#
# Jur van den Berg
# Dept. of Computer Science
# 201 S. Columbia St.
# Frederick P. Brooks, Jr. Computer Science Bldg.
# Chapel Hill, N.C. 27599-3175
# United States of America
#
# <http://gamma.cs.unc.edu/RVO/>
#

set(RVO_HEADERS
	RVOSimulator.h
	vector2.h)

set(RVO_SOURCES
	Agent.cpp
	Agent.h
	Goal.cpp
	Goal.h
	KDTree.cpp
	KDTree.h
	Obstacle.cpp
	Obstacle.h
	RoadmapVertex.cpp
	RoadmapVertex.h
	RVODef.h
	RVOSimulator.cpp)

add_library(rvo ${RVO_HEADERS} ${RVO_SOURCES})

install(FILES ${RVO_HEADERS} DESTINATION include)
install(TARGETS rvo DESTINATION lib)
