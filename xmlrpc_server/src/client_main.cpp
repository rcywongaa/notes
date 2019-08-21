#include <xmlrpc-c/base.hpp>
#include <xmlrpc-c/client_simple.hpp>

int main(int argc, char** argv)
{
    {
        xmlrpc_c::clientSimple client;
        xmlrpc_c::value result;
        client.call("http://localhost:8080/RPC2", "sample.bleh", "n", &result);
    }

    {
        xmlrpc_c::clientSimple client;
        xmlrpc_c::value result;
        client.call("http://localhost:8080/RPC2", "sample.add", "ii", &result, 5, 7);
        int const sum((xmlrpc_c::value_int(result)));
        printf("Result of RPC (sum of 5 and 7): %d\n", sum);
    }
}
