find_package(gazebo REQUIRED)

set (worlds
  empty_underwater.world
)

# Install all the world files
install(FILES ${worlds}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/worlds)
