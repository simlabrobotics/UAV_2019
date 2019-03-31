
project(ALL_APPLICATIONS)

# List of applications
list_subdirectories(_LIST_MODULES_DIRS "${CMAKE_SOURCE_DIR}/demos")
# Add applications
foreach(dir ${_LIST_MODULES_DIRS})
	if( NOT ${dir} STREQUAL "common" )
		add_subdirectory(${dir})
	endif()
endforeach(dir)
