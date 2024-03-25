cmake_minimum_required(VERSION 3.0)
project(cybathlon_feedback)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 std_msgs
       		 dynamic_reconfigure
			 rosneuro_msgs
			 rosneuro_feedback_wheel
			 cybathlon_commons
)

generate_dynamic_reconfigure_options(
	cfg/CybathlonWheel.cfg
)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	std_msgs
	rosneuro_msgs
	rosneuro_feedback_wheel
	cybathlon_commons
  DEPENDS
) 
 
 
###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS}
					include)

add_library(${PROJECT_NAME}	
			src/DoubleThresholdWheel.cpp
			)

target_link_libraries(${PROJECT_NAME} 
					  ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} 
				 ${PROJECT_NAME}_gencfg)

#################
## Executables ##
#################
add_executable(doublethresholdwheel src/doublethresholdwheel.cpp)
target_link_libraries(doublethresholdwheel ${PROJECT_NAME}
								 	 	   ${catkin_LIBRARIES})
