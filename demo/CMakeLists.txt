find_package(Boost)
include_directories("." ${PROJECT_SOURCE_DIR}/lib ${Boost_INCLUDE_DIR})
add_executable(Demo main.cpp)
target_link_libraries (Demo MeshSdf)