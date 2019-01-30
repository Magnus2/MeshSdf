find_package(Boost)
find_package(OpenMP)
include_directories(${Boost_INCLUDE_DIR})

add_library(MeshSdf 
DataStructs.h
MeshAdjacencyTable.cpp
MeshAdjacencyTable.h
MeshLoader.cpp
MeshLoader.h
PointTriangleDistance.cpp
PointTriangleDistance.h
SignedDistance.cpp
SignedDistance.h
UnsignedDistance.cpp
UnsignedDistance.h
Visualize.cpp
Visualize.h
)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

install(TARGETS MeshSdf
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY .
        DESTINATION .
FILES_MATCHING PATTERN "*.h")