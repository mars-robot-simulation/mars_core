cmake_minimum_required(VERSION 3.5)
project(mars_core VERSION 2.0.0 DESCRIPTION "This library contains the core functionality of mars2")

find_package(Boost COMPONENTS serialization)

find_package(Rock)

include(FindPkgConfig)

find_package(lib_manager)
lib_defaults()
define_module_info()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} "${PROJECT_SOURCE_DIR}/cmake")

set(DEFAULT_CONFIG_DIR "${CMAKE_INSTALL_PREFIX}/configuration/mars_default" CACHE STRING "The Default config dir to load")
add_definitions(-DDEFAULT_CONFIG_DIR=\"${DEFAULT_CONFIG_DIR}\")


MACRO(CMAKE_USE_FULL_RPATH install_rpath)
    SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
    SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
    SET(CMAKE_INSTALL_RPATH ${install_rpath})
    SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
ENDMACRO(CMAKE_USE_FULL_RPATH)
CMAKE_USE_FULL_RPATH("${CMAKE_INSTALL_PREFIX}/lib")


#Get linker and compiler flags from pkg-config
pkg_check_modules(PKGCONFIG REQUIRED
#         ode
          lib_manager
#         data_broker
          cfg_manager
          mars_interfaces
          mars_utils
          configmaps
          osg_lines
          envire_core
          envire_types
          mars_ode_physics
          mars_ode_collision
)

include_directories(${PKGCONFIG_INCLUDE_DIRS})
link_directories(${PKGCONFIG_LIBRARY_DIRS})
add_definitions(${PKGCONFIG_CFLAGS_OTHER})  #flags excluding the ones with -I

#add_definitions(-DODE11=1 -DdDOUBLE)
#add_definitions(-DFORWARD_DECL_ONLY=1)

foreach(DIR ${CFG_MANAGER_INCLUDE_DIRS})
    set(ADD_INCLUDES "${ADD_INCLUDES} -I${DIR}")
endforeach(DIR)

include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/src )

set(SOURCES_H
       src/Simulator.hpp
       src/SubWorld.hpp
       src/SimMotor.hpp
       src/MotorManager.hpp
       src/SensorManager.hpp
       src/NodeManager.hpp
       src/JointManager.hpp
       src/JointIDManager.hpp
       src/CollisionManager.hpp
       src/AbsolutePoseExtender.hpp
)
set(SOURCES_SENSORS_H
    src/sensors/CameraSensor.hpp
    src/sensors/RaySensor.hpp
)

set(TARGET_SRC
       src/Simulator.cpp
       src/SubWorld.cpp
       src/SimMotor.cpp
       src/SimJoint.cpp
       src/SimNode.cpp
       src/MotorManager.cpp
       src/SensorManager.cpp
       src/NodeManager.cpp
       src/JointManager.cpp
       src/sensors/CameraSensor.cpp
       src/sensors/RaySensor.cpp
       src/CollisionManager.cpp
       src/registration/PhysicsInterfaceItemRegister.cpp
       src/registration/CollisionInterfaceItemRegister.cpp
       src/registration/DynamicObjectItemRegister.cpp
       src/registration/JointInterfaceItemRegister.cpp
       src/AbsolutePoseExtender.cpp
)

#cmake variables
#configure_file(${CMAKE_SOURCE_DIR}/config.h.in ${CMAKE_BINARY_DIR}/config.h @ONLY)
include_directories("${CMAKE_BINARY_DIR}")
add_library(${PROJECT_NAME} SHARED ${TARGET_SRC})

IF (WIN32)
  set(WIN_LIBS -lwsock32 -lwinmm -lpthread)
#  SET_TARGET_PROPERTIES(mars PROPERTIES LINK_FLAGS -Wl,--stack,0x1000000)
ENDIF (WIN32)

set(_INSTALL_DESTINATIONS
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION ${LIB_INSTALL_DIR}
        ARCHIVE DESTINATION lib
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
            ${PKGCONFIG_LIBRARIES}
            ${WIN_LIBS}
            ${Boost_LIBRARIES}
)



if(WIN32)
  set(LIB_INSTALL_DIR bin) # .dll are in PATH, like executables
else(WIN32)
  set(LIB_INSTALL_DIR lib)
endif(WIN32)

set(_INSTALL_DESTINATIONS
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION ${LIB_INSTALL_DIR}
        ARCHIVE DESTINATION lib
)

# Install the library
install(TARGETS ${PROJECT_NAME} ${_INSTALL_DESTINATIONS})

# Install headers into mars include directory
install(FILES ${SOURCES_H} DESTINATION include/${PROJECT_NAME})
install(FILES ${SOURCES_SENSORS_H} DESTINATION include/${PROJECT_NAME}/sensors)


configure_file(${PROJECT_NAME}.pc.in ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)

# documentation
configure_file(${CMAKE_SOURCE_DIR}/doc/Doxyfile.in ${CMAKE_BINARY_DIR}/doc/Doxyfile @ONLY)
add_custom_target(doc
  COMMAND doxygen ${CMAKE_BINARY_DIR}/doc/Doxyfile
)

