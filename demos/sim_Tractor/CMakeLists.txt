SET(name sim_Tractor)

# import 3rdparty:
SET( RLAB_DEP_LINK_LIBS )

if( RLAB_HAS_KAI )
	list(APPEND ${name}_EXTRA_SRCS		"${KAI_INCLUDE_DIR}/*.h" )
	list(APPEND ${name}_EXTRA_SRCS_NAME 	"KAI" )	
	include_directories("${KAI_INCLUDE_DIR}")	
	set( RLAB_DEP_LINK_LIBS ${RLAB_DEP_LINK_LIBS} optimized ${KAI_LIBRARIES} )
	set( RLAB_DEP_LINK_LIBS ${RLAB_DEP_LINK_LIBS} debug ${KAI_LIBRARIES_DEBUG} )
endif() 

# Add special defines needed by this plugin, if any:
ADD_DEFINITIONS(
	-D_UNICODE
	-D_CRT_SECURE_NO_WARNINGS
	)

INCLUDE_DIRECTORIES( "${CMAKE_SOURCE_DIR}/common" )
INCLUDE_DIRECTORIES( "${CMAKE_SOURCE_DIR}/include" )

#---------------------------------------------
# Macro declared in "MakePlugins.cmake":
#---------------------------------------------
LIST(APPEND ${name}_EXTRA_SRCS
	"${CMAKE_SOURCE_DIR}/include/UAV_conf.h"
	"${CMAKE_SOURCE_DIR}/include/UAV_def.h"
	"${CMAKE_SOURCE_DIR}/include/UAV_hmap.h"
	"${CMAKE_SOURCE_DIR}/include/UAV_protocol.h"
	"${CMAKE_SOURCE_DIR}/common/HeightMap.h"
	"${CMAKE_SOURCE_DIR}/common/HeightMap.cpp"
	"${CMAKE_SOURCE_DIR}/bin/conf_Combine.xdl"
	"${CMAKE_SOURCE_DIR}/bin/models/Mobile/Tractor/devices/camera_v1.0.dml"
	"${CMAKE_SOURCE_DIR}/bin/models/Mobile/Tractor/devices/CombineDrive_v2.0.dml"
	"${CMAKE_SOURCE_DIR}/bin/models/Mobile/Tractor/devices/GPS_v1.0.dml"
	"${CMAKE_SOURCE_DIR}/bin/models/Mobile/Tractor/devices/GRID_TOOL_combine.dml"
	"${CMAKE_SOURCE_DIR}/bin/models/Mobile/Tractor/devices/GRID_v2.0.dml"
	"${CMAKE_SOURCE_DIR}/bin/models/Mobile/Tractor/devices/gyro_v1.0.dml"
	"${CMAKE_SOURCE_DIR}/bin/models/Mobile/Tractor/devices/kinematic_motor_v1.0.dml"
	"${CMAKE_SOURCE_DIR}/bin/models/Mobile/Tractor/devices/PTO_v2.0.dml"
	)
LIST(APPEND ${name}_EXTRA_SRCS_NAME
	"UAV Header"
	"UAV Header"
	"UAV Header"
	"UAV Header"
	"HeightMap"
	"HeightMap"
	"XDL"
	"DML"
	"DML"
	"DML"
	"DML"
	"DML"
	"DML"
	"DML"
	"DML"
	)

make_example (
	# Example name
	${name}
	# Dependencies
	)

#---------------------------------------------
# Install model files & resources:
#---------------------------------------------
install(DIRECTORY "${CMAKE_SOURCE_DIR}/bin/"
	DESTINATION ${RLAB_INSTALL_PREFIX}bin/
#   FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
	)
