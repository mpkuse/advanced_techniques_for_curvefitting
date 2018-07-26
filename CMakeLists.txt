cmake_minimum_required (VERSION 2.8)
project (proto_app)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

  set( CMAKE_CXX_FLAGS "-fpermissive -std=c++11" )


add_executable( helloDemo hello.cpp )
target_link_libraries( helloDemo )

add_executable( hello-ceres hello-ceres.cpp )
target_link_libraries( hello-ceres  ${CERES_LIBRARIES}  )

add_executable( line_fitting line_fitting.cpp )
target_link_libraries( line_fitting  ${CERES_LIBRARIES}  )


add_executable( quadratic_fit quadratic_fit.cpp )
target_link_libraries( quadratic_fit  ${CERES_LIBRARIES}  )


add_executable( align3d3d align3d3d.cpp )
target_link_libraries( align3d3d  ${CERES_LIBRARIES}  )


add_executable( align3d2d align3d2d.cpp )
target_link_libraries( align3d2d  ${CERES_LIBRARIES}  )


add_executable( align3d3d_with_cov align3d3d_with_covariance.cpp )
target_link_libraries( align3d3d_with_cov  ${CERES_LIBRARIES}  )
