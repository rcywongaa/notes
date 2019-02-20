#include "XMLRPCServer.hpp"
#include "functional"
#include "unistd.h"
#include <signal.h>

void sigchld_ignore(int s){
    printf("Ignoring SIGCHLD\n");
}

XMLRPCServer::XMLRPCServer(unsigned int port, std::vector<std::unique_ptr<XMLRPCMethod>> methods) :
    PORT(port),
    runner(std::bind(&XMLRPCServer::run, this)),
    methods(std::move(methods))
{
    // Required if we don't want server to own all signals
    // http://xmlrpc-c.sourceforge.net/doc/libxmlrpc_server_abyss++.html#signals
    signal(SIGCHLD, sigchld_ignore);
}

XMLRPCServer::~XMLRPCServer()
{
    server->terminate();
    runner.join();
}

void XMLRPCServer::run()
{
    xmlrpc_c::registry reg;

    for (int i = 0; i < methods.size(); i++)
    {
        std::string method_name = methods[i]->getName();
        xmlrpc_c::methodPtr const method_ptr(methods[i].release()); // object ownership is passed, manually release()
        reg.addMethod(method_name, method_ptr);
    }

    // serverOwnsSignals is optional, only if we want to handle signals outside serverAbyss
    // http://xmlrpc-c.sourceforge.net/doc/libxmlrpc_server_abyss++.html#signals
    server = std::make_unique<xmlrpc_c::serverAbyss>(xmlrpc_c::serverAbyss::constrOpt().registryP(&reg).portNumber(PORT).serverOwnsSignals(false));
    server->run();
}

