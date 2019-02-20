#include <unistd.h>
#include <signal.h>

#include "XMLRPCServer.hpp"

using namespace std;

class sampleAddMethod : public XMLRPCMethod {
    public:
        sampleAddMethod() {
            // signature and help strings are documentation -- the client
            // can query this information with a system.methodSignature and
            // system.methodHelp RPC.
            this->_signature = "i:ii";
            // method's result and two arguments are integers
            this->_help = "This method adds two integers together";

            this->name = "sample.add";
        }
        void execute(xmlrpc_c::paramList const& paramList,
                    xmlrpc_c::value *   const  retvalP) {

                int const addend(paramList.getInt(0));
                int const adder(paramList.getInt(1));

                paramList.verifyEnd(2);

                *retvalP = xmlrpc_c::value_int(addend + adder);
        };
};

volatile bool is_continue = true;

void sigint_handler(int s){
    printf("--- SIGINT ---\n");
    is_continue = false;
}

int main(int const, const char ** const) {
    signal(SIGINT, sigint_handler);
    std::vector<std::unique_ptr<XMLRPCMethod>> methods;
    methods.push_back(std::make_unique<sampleAddMethod>());
    XMLRPCServer server(8080, std::move(methods));
    while (is_continue);
}
