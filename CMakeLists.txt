cmake_minimum_required(VERSION 2.8.3)
project(kimera_vio_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(cbs REQUIRED)
find_package(aria_viz REQUIRED)
find_package(OpenCV REQUIRED)

### System Dependencies
find_package(Boost REQUIRED COMPONENTS
  serialization system filesystem thread date_time regex timer chrono)
if(NOT TARGET Boost::boost)
  add_library(Boost::boost INTERFACE IMPORTED)
  set_target_properties(Boost::boost PROPERTIES
  INTERFACE_LINK_LIBRARIES "${Boost_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}")
endif()

catkin_simple()

cs_add_library(${PROJECT_NAME}
  # Include files are just here for IDEs
  include/kimera_vio_ros/KimeraVioRos.h
  include/kimera_vio_ros/RosBagDataProvider.h
  include/kimera_vio_ros/RosDataProviderInterface.h
  include/kimera_vio_ros/RosDisplay.h
  include/kimera_vio_ros/RosOnlineDataProvider.h
  include/kimera_vio_ros/RosPublishers.h
  include/kimera_vio_ros/RosRerunVisualizer.h
  include/kimera_vio_ros/RosVisualizer.h
  include/kimera_vio_ros/utils/UtilsRos.h
  include/kimera_vio_ros/utils/CsvPublisher.h
  src/KimeraVioRos.cpp
  src/RosDataProviderInterface.cpp
  src/RosBagDataProvider.cpp
  src/RosOnlineDataProvider.cpp
  src/RosDisplay.cpp
  src/RosRerunVisualizer.cpp
  src/RosVisualizer.cpp
  src/RosLoopClosureVisualizer.cpp
  src/LcdRegistrationServer.cpp
  src/utils/UtilsRos.cpp
  src/utils/CsvPublisher.cpp
)
target_include_directories(${PROJECT_NAME}
  SYSTEM BEFORE PUBLIC
    ${OpenCV_INCLUDE_DIRS}
)
set(GTSAM_VENDORED_EIGEN_INCLUDE "${GTSAM_INCLUDE_DIR}/gtsam/3rdparty/Eigen")
if(EXISTS "${GTSAM_VENDORED_EIGEN_INCLUDE}/Eigen/Dense")
  target_include_directories(${PROJECT_NAME}
    SYSTEM BEFORE PUBLIC
      ${GTSAM_VENDORED_EIGEN_INCLUDE}
  )
endif()
target_link_libraries(${PROJECT_NAME} kimera_vio cbs aria_viz ${OpenCV_LIBRARIES})

cs_add_executable(${PROJECT_NAME}_node
  src/KimeraVioRosNode.cpp
)
target_include_directories(${PROJECT_NAME}_node
  SYSTEM BEFORE PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)
if(EXISTS "${GTSAM_VENDORED_EIGEN_INCLUDE}/Eigen/Dense")
  target_include_directories(${PROJECT_NAME}_node
    SYSTEM BEFORE PRIVATE
      ${GTSAM_VENDORED_EIGEN_INCLUDE}
  )
endif()
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

cs_add_executable(csv_publisher_node
  src/utils/CsvPublisherNode.cpp
)
target_include_directories(csv_publisher_node
  SYSTEM BEFORE PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)
if(EXISTS "${GTSAM_VENDORED_EIGEN_INCLUDE}/Eigen/Dense")
  target_include_directories(csv_publisher_node
    SYSTEM BEFORE PRIVATE
      ${GTSAM_VENDORED_EIGEN_INCLUDE}
  )
endif()
target_link_libraries(csv_publisher_node ${PROJECT_NAME})

cs_add_executable(rerun_topic_visualizer_node
  src/RerunTopicVisualizerNode.cpp
)
target_include_directories(rerun_topic_visualizer_node
  SYSTEM BEFORE PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)
if(EXISTS "${GTSAM_VENDORED_EIGEN_INCLUDE}/Eigen/Dense")
  target_include_directories(rerun_topic_visualizer_node
    SYSTEM BEFORE PRIVATE
      ${GTSAM_VENDORED_EIGEN_INCLUDE}
  )
endif()
target_link_libraries(rerun_topic_visualizer_node ${PROJECT_NAME})

cs_add_executable(artifact_rerun_visualizer_node
  src/ArtifactRerunVisualizerNode.cpp
)
target_include_directories(artifact_rerun_visualizer_node
  SYSTEM BEFORE PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)
if(EXISTS "${GTSAM_VENDORED_EIGEN_INCLUDE}/Eigen/Dense")
  target_include_directories(artifact_rerun_visualizer_node
    SYSTEM BEFORE PRIVATE
      ${GTSAM_VENDORED_EIGEN_INCLUDE}
  )
endif()
target_link_libraries(artifact_rerun_visualizer_node ${PROJECT_NAME})

catkin_add_gtest(testKimeraVioRos
  test/testKimeraVioRos.cpp
)
target_include_directories(testKimeraVioRos
  SYSTEM BEFORE PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(testKimeraVioRos ${PROJECT_NAME})

cs_install()

cs_export()
