cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_BUILD_TYPE Debug)
project(HelloWorld)

include_directories(include)

#find_package (Threads REQUIRED)

add_executable(helloworld src/main.cpp)
target_link_libraries(helloworld
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
 )

