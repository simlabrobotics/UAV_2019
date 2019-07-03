SET( name "rDeviceTractorDrive" )

# import 3rdparty 
SET( RLAB_DEP_LINK_LIBS )

# Add special defines needed by this plugin, if any:
ADD_DEFINITIONS(
	-DR_EXPORTS
	-D_UNICODE
	-D_CRT_SECURE_NO_WARNINGS
	)

INCLUDE_DIRECTORIES( "${CMAKE_SOURCE_DIR}/common" )
INCLUDE_DIRECTORIES( "${CMAKE_SOURCE_DIR}/include" )

#---------------------------------------------
# Macro declared in "MakePlugins.cmake":
#---------------------------------------------
LIST(APPEND ${name}_EXTRA_SRCS
	"${CMAKE_SOURCE_DIR}/common/HeightMap.h"
	"${CMAKE_SOURCE_DIR}/common/HeightMap.cpp"
	)
LIST(APPEND ${name}_EXTRA_SRCS_NAME
	"HeightMap"
	"HeightMap"
	)

make_example_plugin(
	# Plugin name
	${name}
	# Dependencies
	)

#---------------------------------------------
# Install device drivers:
#---------------------------------------------
install(TARGETS "${name}"
	RUNTIME DESTINATION ${RLAB_INSTALL_PREFIX}bin/models/devices/
	)