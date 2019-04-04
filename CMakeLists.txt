SET( name UAV )
SET( RLAB_ROOT_DIR "C:/Program Files (x86)/RLAB" CACHE PATH "Where RLAB is installed?" )

PROJECT( ${name} )
if( MSVC )
	cmake_minimum_required(VERSION 2.8.0)
else( MSVC )
	cmake_minimum_required(VERSION 2.6.3)
endif( MSVC )


message( STATUS "-----------------------------------------------------------------" )
message( STATUS "Project name: ${CMAKE_PROJECT_NAME}" )

# ----------------------------------------------------------------------------
#  Loads the current version number:
# ----------------------------------------------------------------------------
file( READ "${RLAB_ROOT_DIR}/version.txt" RLAB_VERSION_FILE )
string( REGEX MATCHALL "[0-9]+" RLAB_VERSION_PARTS "${RLAB_VERSION_FILE}" )
list( GET RLAB_VERSION_PARTS 0 RLAB_VERSION_NUMBER_MAJOR )
list( GET RLAB_VERSION_PARTS 1 RLAB_VERSION_NUMBER_MINOR )
list( GET RLAB_VERSION_PARTS 2 RLAB_VERSION_NUMBER_PATCH )
set( RLAB_VERSION_NUMBER "${RLAB_VERSION_NUMBER_MAJOR}.${RLAB_VERSION_NUMBER_MINOR}.${RLAB_VERSION_NUMBER_PATCH}" )
# Build a three digits version code, eg. 0.5.1 -> 051,  1.2.0 -> 120
set( RLAB_VERSION_CODE "0x${RLAB_VERSION_NUMBER_MAJOR}${RLAB_VERSION_NUMBER_MINOR}${RLAB_VERSION_NUMBER_PATCH}" )
message( STATUS "Found rlab ${RLAB_VERSION_NUMBER}" )

# ----------------------------------------------------------------------------
# The root directory for all RLAB libraries/modules:
# ----------------------------------------------------------------------------
set( RLAB_SOURCE_DIR "${RLAB_ROOT_DIR}" CACHE INTERNAL "" )
set( RLAB_INCLUDE_DIR "${RLAB_ROOT_DIR}/include" CACHE INTERNAL "" )
set( RLAB_LIBRARY_DIR "${RLAB_ROOT_DIR}/lib" CACHE INTERNAL "" )
set( RLAB_3RDPARTY_DIR "${RLAB_ROOT_DIR}/3rdparty" CACHE INTERNAL "" )
set( RLAB_EXAMPLES_DIR "${CMAKE_SOURCE_DIR}" CACHE INTERNAL "" )
set( RLAB_INSTALL_PREFIX "${RLAB_ROOT_DIR}/" CACHE INTERNAL "" )

# ----------------------------------------------------------------------------
#  rlab include/library directories:
# ----------------------------------------------------------------------------
include_directories( "${RLAB_INCLUDE_DIR}" )
include_directories( "${RLAB_3RDPARTY_DIR}/tcl_5.3.0" )
link_directories( "${RLAB_LIBRARY_DIR}" )
include_directories( "${CMAKE_SOURCE_DIR}/include" )

# ---------------------------------------------------------------------------
# Auxiliary macros:
# ---------------------------------------------------------------------------
include( "${RLAB_ROOT_DIR}/cmakemodules/FindEigen.cmake" )
include( "${RLAB_ROOT_DIR}/cmakemodules/FindXerces.cmake" )
include( "${RLAB_ROOT_DIR}/cmakemodules/FindKai.cmake" )
include( "${RLAB_ROOT_DIR}/cmakemodules/FindTCL.cmake" )
include( "${RLAB_ROOT_DIR}/cmakemodules/FindIPP.cmake" )
include( "${RLAB_ROOT_DIR}/cmakemodules/FindIPPJpeg.cmake" )
include( "${RLAB_ROOT_DIR}/cmakemodules/FindChartDirector.cmake" )
include( "${RLAB_ROOT_DIR}/cmakemodules/FindOpenCV.cmake" )
include( "${RLAB_ROOT_DIR}/cmakemodules/FilterFileLists.cmake" REQUIRED ) 
include( "${RLAB_ROOT_DIR}/cmakemodules/DeclareAppDependencies.cmake" REQUIRED )
include( "${RLAB_ROOT_DIR}/cmakemodules/MakeExamples.cmake" REQUIRED )
include( "${RLAB_ROOT_DIR}/cmakemodules/UtilsMacros.cmake" REQUIRED )

# ----------------------------------------------------------------------------
#  Adds rlab user projects:
# ----------------------------------------------------------------------------
add_subdirectory(devices)
add_subdirectory(graphics)
#add_subdirectory(simulations)
#add_subdirectory(controls)
add_subdirectory(demos)
add_subdirectory(tools)

# ----------------------------------------------------------------------------
#  Install examples:
# ----------------------------------------------------------------------------
set(INSTALL_RLAB_EXAMPLES			OFF CACHE BOOL "Install into rlab folder")
