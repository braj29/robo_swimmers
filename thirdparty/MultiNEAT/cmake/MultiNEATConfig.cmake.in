get_filename_component(MultiNEAT_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(CMakeFindDependencyMacro)

list(APPEND CMAKE_MODULE_PATH ${MultiNEAT_CMAKE_DIR})

# NOTE Had to use find_package because find_dependency does not support COMPONENTS or MODULE until 3.8.0
#find_dependency(Boost 1.55 REQUIRED COMPONENTS date_time
#        system
#        filesystem
#        serialization)
find_package(Boost 1.55 REQUIRED COMPONENTS
        date_time
        system
        filesystem
        serialization)
find_package(Threads REQUIRED)

list(REMOVE_AT CMAKE_MODULE_PATH -1)

if(NOT TARGET MultiNEAT::MultiNEAT)
    include("${MultiNEAT_CMAKE_DIR}/MultiNEATTargets.cmake")
endif()

set(MultiNEAT_LIBRARIES MultiNEAT::MultiNEAT)
