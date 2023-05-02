cmake_minimum_required(VERSION 3.0)
project(cybathlon_feedback)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs
			 rosneuro_msgs
			 geometry_msgs
			 )

find_library(NEURODRAW_LIBRARIES neurodraw REQUIRED)
find_path(NEURODRAW_INCLUDE_DIRS neurodraw/Shape.h REQUIRED)

find_library(NEUROCHRONO_LIBRARIES neurochrono REQUIRED)
find_path(NEUROCHRONO_INCLUDE_DIRS neurochrono/Timer.h REQUIRED)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	std_msgs
	rosneuro_msgs
	geometry_msgs
  DEPENDS
)


###########
## Build ##
###########

include_directories(include 
					${catkin_INCLUDE_DIRS} 
					${NEURODRAW_INCLUDE_DIRS}
					${NEUROCHRONO_INCLUDE_DIRS})

add_library(${PROJECT_NAME}	
			src/draw/smr_dweel.cpp
			src/draw/smr_dweel_bars.cpp
			)

target_link_libraries(${PROJECT_NAME} 
					  ${catkin_LIBRARIES}
					  ${NEURODRAW_LIBRARIES}
					  ${NEUROCHRONO_LIBRARIES})

add_dependencies(${PROJECT_NAME} 
				 ${catkin_EXPORTED_TARGETS}
)

#################
## Executables ##
#################
add_executable(neurowheel src/neurowheel.cpp)
target_link_libraries(neurowheel ${PROJECT_NAME}
								 ${catkin_LIBRARIES}
					  			 ${NEURODRAW_LIBRARIES}
					  			 ${NEUROCHRONO_LIBRARIES})
