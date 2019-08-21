cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_BUILD_TYPE Debug)
project(XMLRPCServer)

include_directories(include)

find_package (Threads REQUIRED)

add_library(my_xmlrpc_server src/XMLRPCServer.cpp)
target_include_directories(my_xmlrpc_server PUBLIC ./include)
target_link_libraries(my_xmlrpc_server
    xmlrpc_server++
    xmlrpc_server_abyss++
    xmlrpc++
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(run_xmlrpc_server src/main.cpp)
target_link_libraries(run_xmlrpc_server my_xmlrpc_server)

add_executable(my_xmlrpc_client src/client_main.cpp)
target_link_libraries(my_xmlrpc_client
    xmlrpc_client++
    xmlrpc_client
    xmlrpc++
)
