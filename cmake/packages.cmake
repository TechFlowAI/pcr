# eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

# yaml-cpp
# find_package(yaml-cpp REQUIRED)
# include_directories(${yaml-cpp_INCLUDE_DIRS})

# set(third_party_libs
#     ${PCL_LIBRARIES}
#     ${yaml-cpp_LIBRARIES}
#     yaml-cpp
# )

set(third_party_libs
    ${PCL_LIBRARIES}
)