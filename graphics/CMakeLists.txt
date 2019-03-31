
PROJECT(ALL_GRAPHICS)

# List of graphics plugins
list_subdirectories(_LIST_MODULES_DIRS "${CMAKE_SOURCE_DIR}/graphics")
# Add graphics plug-ins
foreach(dir ${_LIST_MODULES_DIRS})
	if( NOT ${dir} STREQUAL "common" )
		add_subdirectory(${dir})
	endif()
endforeach(dir)
