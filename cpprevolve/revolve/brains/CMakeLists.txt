set (CMAKE_CXX_STANDARD 17)

set(CONTROLLER_SRCS
    controller/sensors/AngleToTargetDetector.cpp
    controller/DifferentialCPG.cpp
)

set(LEARNER_SRCS
    learner/BayesianOptimizer.cpp
)

# PKG-CONFIG
find_package(PkgConfig REQUIRED)

# Find Boost
find_package(Boost REQUIRED COMPONENTS system)

# Find Eigen3 - A lightweight C++ template library for vector and matrix math
find_package(Eigen3 REQUIRED)
find_package(MultiNEAT REQUIRED)

# These dependencies are required for the AngleToTargetDetector Sensor
#find_package(OpenCV REQUIRED)
#find_package(raspicam REQUIRED) #only on the raspberry side

find_package(MultiNEAT REQUIRED)

# Find NLOpt - Non Linear Optimization
pkg_check_modules(NLOpt REQUIRED nlopt>=2.4)

# Find Limbo - LIbrary for Model-Based Optimization
set(LIMBO_DIR ${CMAKE_SOURCE_DIR}/thirdparty/limbo)
set(LIMBO_DEFINES USE_NLOPT)

add_library(revolve-controllers SHARED ${CONTROLLER_SRCS})
add_library(revolve-learners SHARED ${LEARNER_SRCS})

target_include_directories(revolve-controllers
        PUBLIC ${EIGEN3_INCLUDE_DIR}
        PUBLIC ${Boost_INCLUDE_DIRS}
        PUBLIC ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(revolve-learners
        PUBLIC ${EIGEN3_INCLUDE_DIR}
        PUBLIC ${Boost_INCLUDE_DIRS}
        PUBLIC ${LIMBO_DIR}/src
        PUBLIC ${NLOpt_INCLUDE_DIRS})

target_compile_definitions(revolve-learners
        PUBLIC USE_NLOPT=1
)

target_link_libraries(revolve-controllers
        PUBLIC MultiNEAT::MultiNEAT
        ${OpenCV_LIBS}
        )

target_link_libraries(revolve-learners
        revolve-controllers
        MultiNEAT::MultiNEAT)

install(TARGETS revolve-controllers revolve-learners
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib)
