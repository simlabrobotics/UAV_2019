SET(name app_UAVClient)

# import 3rdparty:
SET( APP_DEP_LINK_LIBS )

if( RLAB_HAS_KAI )
	list(APPEND ${name}_EXTRA_SRCS		"${KAI_INCLUDE_DIR}/*.h" )
	list(APPEND ${name}_EXTRA_SRCS_NAME 	"KAI" )	
	include_directories("${KAI_INCLUDE_DIR}")	
	set( APP_DEP_LINK_LIBS ${APP_DEP_LINK_LIBS} optimized ${KAI_LIBRARIES} )
	set( APP_DEP_LINK_LIBS ${APP_DEP_LINK_LIBS} debug ${KAI_LIBRARIES_DEBUG} )
endif() 

# add special defines needed, if any:
ADD_DEFINITIONS(
	-D_UNICODE
	-D_CRT_SECURE_NO_WARNINGS
	)

# add source and header files and make executable:
ADD_EXECUTABLE(${name}
	ClientSocket.h
	ClientSocket.cpp
	app_UAVClient.cpp
	)

TARGET_LINK_LIBRARIES(${name} 
	${APP_DEP_LINK_LIBS}
	ws2_32.lib
	)

#---------------------------------------------
# Install executable:
#---------------------------------------------
install(TARGETS "${name}"
	RUNTIME DESTINATION ${RLAB_INSTALL_PREFIX}bin/
	)
