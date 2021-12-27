cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(RPiEnvironmentalMonitorSystem VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# To build the hardware library
add_subdirectory (hardware)
# To build the sensors' object files
add_subdirectory (sensors)

find_library(cpp_tools hardware PATHS ${PROJECT_SOURCE_DIR}/../../C-Projects/Tools)
find_library(hwlib hardware PATHS ${PROJECT_SOURCE_DIR}/hardware)

configure_file(EnvironmentMonitor.h.in ${PROJECT_SOURCE_DIR}/EnvironmentMonitor.h)

add_executable( EnvironmentMonitor
    EnvironmentMonitor.cpp
    DataStore.cpp
    MasterTask.cpp
    )

target_include_directories(EnvironmentMonitor PUBLIC
    /opt/homebrew/include
    ${PROJECT_SOURCE_DIR}/../../C-Projects/Tools/include
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/hardware/include
    ${PROJECT_SOURCE_DIR}/sensors/include
    )

target_link_libraries(EnvironmentMonitor PUBLIC ${PROJECT_SOURCE_DIR}/../../C-Projects/Tools/libcpp_tools.a)

install(TARGETS EnvironmentMonitor DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}" )

#foreach(sourceFile
#  avl1.cpp
#  avl-tree.cpp
#  chrono-ex.cpp
#  cli_example.cpp
#  condvar.cpp
#  fnTimeout.cpp
#  functor.cpp
#  gcdList.cpp
#  Mediator.cpp
#  msd.cpp
#  ObserverObservable.cpp
#  palindrome.cpp
#  pub_sub_cpp.cpp
#  rat.cpp
#  read_config_file.cpp
#  removeSpaces.cpp
#  seive.cpp
#  shmem-test.cpp
#  SigHandler.cpp
#  streams.cpp
#  tConfigure.cpp
#  TestLog.cpp
#  tFilesystem.cpp
#  timedemo.cpp
#  tPubSub.cpp
#  tryCpp.cpp
#  tryThread.cpp
#  tryVector.cpp
#  ts.cpp
#  updateServers.cpp
#  variadic-tuple.cpp
#  waitfor.cpp )
#    # Use string replace to cut off '.cpp'
#    string(REPLACE ".cpp" "" target ${sourceFile} )
#    add_executable(${target} ${sourceFile} )
#    target_include_directories(${target} PRIVATE ${PROJECT_SOURCE_DIR}/../Tools/include)
#    target_link_libraries(${target} PUBLIC cppTools pthread)
#    install(TARGETS ${target} DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}" )
#endforeach(sourceFile "")
#
#foreach(sourceFile
#  socket-client.c
#  socket-server.c )
#    string(REPLACE ".c" "" target ${sourceFile} )
#    add_executable(${target} ${sourceFile} )
#    target_link_libraries(${target} PUBLIC pthread)
#    install(TARGETS ${target} DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}" )
#endforeach(sourceFile "")
