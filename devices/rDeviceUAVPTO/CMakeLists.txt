SET( name "rDeviceUAVPTO" )

# import 3rdparty 
SET( RLAB_DEP_LINK_LIBS )

# Add special defines needed by this plugin, if any:
SET(MY_DEFS )
IF(MY_DEFS) # If not empty
	ADD_DEFINITIONS("-D${MY_DEFS}")
ENDIF(MY_DEFS)

#---------------------------------------------
# Macro declared in "MakePlugins.cmake":
#---------------------------------------------
make_example_plugin(
	# Plugin name
	${name}
	# Dependencies
	)
