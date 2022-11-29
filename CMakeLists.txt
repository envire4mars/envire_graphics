project(envire_graphics)
set(PROJECT_VERSION 1.0)
set(PROJECT_DESCRIPTION "Plugin")
cmake_minimum_required(VERSION 2.6)
include(FindPkgConfig)
include(CheckIncludeFileCXX)

find_package(lib_manager)
lib_defaults()
define_module_info()
set(CMAKE_CXX_STANDARD 14)

set (QT_USE_QTOPENGL TRUE)
setup_qt()


pkg_check_modules(OSGQT openscenegraph-osgQt5)
if(OSGQT_FOUND)
find_package(OpenSceneGraph REQUIRED osgManipulator osgViewer osgFX osgShadow osgParticle osgTerrain osgDB osgGA osgWidget osgText osgUtil)

include_directories(${OSGQT_INCLUDE_DIRS})
link_directories(${OSGQT_LIBRARY_DIRS})
add_definitions(${OSGQT_CFLAGS_OTHER})  #cflags without -I

else()
find_package(OpenSceneGraph REQUIRED osgManipulator osgViewer osgFX osgShadow osgParticle osgTerrain osgDB osgGA osgWidget osgText osgUtil osgQt)
endif()

include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
link_directories(${OPENSCENEGRAPH_LIBRARY_DIRS})


set (CMAKE_REQUIRED_INCLUDES ${CMAKE_REQUIRED_INCLUDES} ${OPENSCENEGRAPH_INCLUDE_DIRS})
CHECK_INCLUDE_FILE_CXX("osg/Version" HAVE_OSG_VERSION_H)
if(${HAVE_OSG_VERSION_H})
    ADD_DEFINITIONS(-DHAVE_OSG_VERSION_H)
endif()

pkg_check_modules(PKGCONFIG REQUIRED
                lib_manager
                data_broker
                mars_interfaces
                mars_graphics
                mars_sim
                envire_core
                smurf
                envire_managers
                #maps
                #maps-viz
                )

include_directories(${PKGCONFIG_INCLUDE_DIRS})
link_directories(${PKGCONFIG_LIBRARY_DIRS})
add_definitions(${PKGCONFIG_CFLAGS_OTHER})  #flags excluding the ones with -I


add_subdirectory(src)
add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME}
                      ${PKGCONFIG_LIBRARIES}
                      ${OPENSCENEGRAPH_LIBRARIES}
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


# Install the library into the lib folder
install(TARGETS ${PROJECT_NAME} ${_INSTALL_DESTINATIONS})

# Install headers into mars include directory
install(FILES ${HEADERS} DESTINATION include/mars/plugins/${PROJECT_NAME})

# Prepare and install necessary files to support finding of the library
# using pkg-config
configure_file(${PROJECT_NAME}.pc.in ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)

if (${USE_QT5})
qt5_use_modules(${PROJECT_NAME} Widgets OpenGL)
endif (${USE_QT5})


find_package(Rock)
set(ROCK_USE_CXX11 TRUE)
rock_init(envire_graphics 0.1)
rock_standard_layout()


find_package(Boost COMPONENTS system filesystem REQUIRED)
target_link_libraries(envire_graphics
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${QT_LIBRARIES}
  )

