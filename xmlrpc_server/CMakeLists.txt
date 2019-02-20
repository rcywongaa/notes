cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_BUILD_TYPE Debug)
project(XMLRPCServer)

include_directories(include)

find_package (Threads REQUIRED)

add_library(t2_xmlrpc_server src/XMLRPCServer.cpp)
target_link_libraries(t2_xmlrpc_server
    xmlrpc_server++
    xmlrpc_server
    xmlrpc_server_abyss++
    xmlrpc_server++
    xmlrpc_server_abyss
    xmlrpc_server
    xmlrpc_abyss
    pthread
    xmlrpc_client++
    xmlrpc_client
    xmlrpc++
    xmlrpc
    xmlrpc_xmlparse
    xmlrpc_xmltok
    xmlrpc_util
    curl
    xmlrpc_packetsocket
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(xmlrpc_test src/main.cpp)
target_link_libraries(xmlrpc_test t2_xmlrpc_server)
