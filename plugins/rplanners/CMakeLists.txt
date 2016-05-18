###########################################
# rplanners openrave plugin
###########################################
add_subdirectory(ParabolicPathSmooth)
add_library(rplanners SHARED constraintparabolicsmoother.cpp cubicretimer.cpp graspgradient.cpp linearretimer.cpp linearsmoother.cpp mergewaypoints.cpp parabolicretimer.cpp parabolicsmoother.cpp linearshortcutadvanced.cpp randomized-astar.cpp rplanners.h rplanners.cpp rrt.h kinorrt.h workspacetrajectorytracker.cpp)
## include(CheckCXXCompilerFlag)
## CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
## if(COMPILER_SUPPORTS_CXX11)
##     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
## else()
##     message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support")
## endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

target_link_libraries(rplanners libopenrave ParabolicPathSmooth)
set_target_properties(rplanners PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}" LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS rplanners DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${PLUGINS_BASE})
