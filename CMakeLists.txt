include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${mt_DIR}
                    ${OMPL_INCLUDE_DIRS})

add_executable( DualGrasp bimanual.cpp kauthamshell.cpp rob.cpp UR5_kinematics.cpp )
target_link_libraries( DualGrasp libkautham  ${COIN_LIBRARY} ${Boost_LIBRARIES}  ${PUGI_XML_LIBRARIES})
