cmake_minimum_required (VERSION 3.0)
project (MeshSdf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Parallel compilation.
if(WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4250")
endif()

set(CMAKE_DEBUG_POSTFIX "d")

add_subdirectory(lib)
add_subdirectory(demo)