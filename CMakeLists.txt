cmake_minimum_required(VERSION 3.14)
project(mpc_lqr_bicycle CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


#Options
option(USE_SYSTEM_EIGEN "Use system-install Eigen3" ON)
option(ENABLE_YAML "Enable yaml-cpp config loader" ON)

#Eigen
set(EIGEN3_FOUND FALSE)
if(USE_SYSTEM_EIGEN)
    find_package(Eigen3 3.3 QUIET)
    if(Eigen3_FOUND OR EIGEN3_FOUND)
        set(EIGEN3_FOUND TRUE)
        message(STATUS "Found Eigen3: ${EIGEN3_VERSION}")
    else()
        message(WARNING "Eigen3 not found. Please install Eigen3 or set USE_SYSTEM_EIGEN=OFF")
    endif()
else()
    if(EXISTS ${CMAKE_SOURCE_DIR}/third_party/Eigen)
        include_directories(${CMAKE_SOURCE_DIR}/third_party/Eigen)
        set(EIGEN3_FOUND TRUE)
    else()
        message(FATAL_ERROR "Eigen3 not found in third_party. Please install Eigen3 or set USE_SYSTEM_EIGEN=ON")
    endif()
endif()

if(NOT EIGEN3_FOUND)
    message(FATAL_ERROR "Eigen3 is required but not found!")
endif()

#yaml.cpp (optional)
if(ENABLE_YAML)
    find_package(yaml-cpp QUIET)
endif()

add_library(core
    src/bicycle_model.cpp
    src/lqr.cpp
    src/mpc.cpp
    src/trajectory.cpp
    src/util.cpp
)

target_include_directories(core PUBLIC include)

if(USE_SYSTEM_EIGEN AND EIGEN3_FOUND)
    if(TARGET Eigen3::Eigen)
        target_link_libraries(core PUBLIC Eigen3::Eigen)
    elseif(EIGEN3_INCLUDE_DIR)
        target_include_directories(core PUBLIC ${EIGEN3_INCLUDE_DIR})
    else()
        message(FATAL_ERROR "Eigen3 found but cannot determine include directory")
    endif()
endif()

if(ENABLE_YAML AND yaml-cpp_FOUND)
    target_compile_definitions(core PUBLIC HAVE_YAML)
    target_link_libraries(core PUBLIC yaml-cpp)
endif()

add_executable(sim src/main.cpp)
target_link_libraries(sim PRIVATE core)

install(TARGETS sim RUNTIME DESTINATION bin)


