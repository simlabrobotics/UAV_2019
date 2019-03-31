
PROJECT(ALL_DEVICES)

# List of device plug-ins
list_subdirectories(_LIST_MODULES_DIRS "${CMAKE_SOURCE_DIR}/devices")
# Add device plug-ins
foreach(dir ${_LIST_MODULES_DIRS})
	if( NOT ${dir} STREQUAL "common" )
		add_subdirectory(${dir})
	endif()
endforeach(dir)
