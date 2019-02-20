#include <memory>
#include <xmlrpc-c/base.hpp>
#include <xmlrpc-c/registry.hpp>
#include <xmlrpc-c/server_abyss.hpp>

#include <vector>
#include <thread>

class XMLRPCMethod : public xmlrpc_c::method
{
    public:
        std::string getName()
        {
            return name;
        };

        ~XMLRPCMethod()
        {};

    protected:
        std::string name;
};

class XMLRPCServer
{
    public:
        XMLRPCServer(unsigned int port, std::vector<std::unique_ptr<XMLRPCMethod>> methods);
        ~XMLRPCServer();

    private:
        bool is_continue;
        const unsigned int PORT;
        std::thread runner;
        std::vector<std::unique_ptr<XMLRPCMethod>> methods;
        std::unique_ptr<xmlrpc_c::serverAbyss> server;
        void run();
};

