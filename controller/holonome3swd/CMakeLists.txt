include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources Holonome3SwD_k_Position2d.cc)
SET (headers Holonome3SwD_k_Position2d.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
