add_library(${PROJECT_NAME}.fpfh
    fpfh.cc
    fpfh_manager.cc
    feature_matcher.cc)
target_link_libraries(${PROJECT_NAME}.fpfh
    ${third_party_libs})

add_executable(run_fpfh run_fpfh.cc)
target_link_libraries(run_fpfh
    ${PROJECT_NAME}.fpfh
    ${third_party_libs})
