cmake_minimum_required(VERSION 3.15)

# set the project name and version
project(EnvironmentMonitor VERSION 1.5)

configure_file(EnvironmentMonitor.h.in ${PROJECT_SOURCE_DIR}/EnvironmentMonitor.h)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(PROJECT_BINARY_DIR ${PROJECT_SOURCE_DIR}/bin)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

# To build the hardware library
add_subdirectory (hardware)
# To build the sensors' object files
add_subdirectory (sensors)
# To build the user interface
add_subdirectory (ui)

link_directories(${PROJECT_SOURCE_DIR}/hardware ${PROJECT_SOURCE_DIR}/../../C-Projects/Tools/bin)

find_library(cppTools cpp_tools PATHS ${PROJECT_SOURCE_DIR}/../../C-Projects/Tools/bin)
find_library(algobsec algobsec PATHS ${PROJECT_SOURCE_DIR}/../hardware)

add_executable(EnvironmentMonitor
    EnvironmentMonitor.cpp
    DataStore.cpp
    MasterTask.cpp
)

target_link_libraries(EnvironmentMonitor PUBLIC pthread ${cppTools} sensors ui hardware algobsec stdc++fs )

target_include_directories(EnvironmentMonitor PUBLIC
    /opt/homebrew/include
    ${PROJECT_SOURCE_DIR}/../../C-Projects/Tools/include
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/hardware/include
    ${PROJECT_SOURCE_DIR}/sensors/include
    ${PROJECT_SOURCE_DIR}/ui/include
)

install(TARGETS EnvironmentMonitor DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}" )
