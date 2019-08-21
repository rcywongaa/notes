/**
 * @see http://xmlrpc-c.sourceforge.net/doc/#cpplibraries
 * @see http://xmlrpc-c.sourceforge.net/doc/libxmlrpc++.html
 * @see http://xmlrpc-c.sourceforge.net/doc/libxmlrpc_server_abyss++.html
 * @see https://sourceforge.net/p/xmlrpc-c/code/HEAD/tree/trunk/examples/cpp/xmlrpc_sample_add_server.cpp
 *
 * Method Signature:
 * @see http://xmlrpc-c.sourceforge.net/doc/libxmlrpc_server.html#xmlrpc_registry_add_method3
 */

#ifndef XMLRPC_SERVER_INCLUDED
#define XMLRPC_SERVER_INCLUDED

#include <memory>
#include <xmlrpc-c/base.hpp>
#include <xmlrpc-c/registry.hpp>
#include <xmlrpc-c/server_abyss.hpp>

#include <vector>
#include <thread>

/**
 * @brief Class used to create methods for passing to @c XMLRPCServer
 *
 * Extends @c xmlrpc_c::method to include @c name variable
 * since method name for original @c xmlrpc_c::method must be passed in separately
 * which made it less self-encapsulating
 *
 * Usage example:
 * @code
 * class sampleAddMethod : public XMLRPCMethod {
 *     public:
 *         sampleAddMethod() {
 *             // signature and help strings are documentation -- the client
 *             // can query this information with a system.methodSignature and
 *             // system.methodHelp RPC.
 *             this->_signature = "i:ii";
 *             // method's result and two arguments are integers
 *             this->_help = "This method adds two integers together";
 *
 *             this->name = "sample.add";
 *         }
 *         void execute(xmlrpc_c::paramList const& paramList,
 *                     xmlrpc_c::value *   const  retvalP) override {
 *
 *                 int const addend(paramList.getInt(0));
 *                 int const adder(paramList.getInt(1));
 *
 *                 paramList.verifyEnd(2);
 *
 *                 *retvalP = xmlrpc_c::value_int(addend + adder);
 *         };
 * };
 * @endcode
 */

// FIXME: _signature, _help, name should be compulsory constructor parameters
class XMLRPCMethod : public xmlrpc_c::method
{
    public:
        std::string getName()
        {
            return name;
        };

        virtual void execute(xmlrpc_c::paramList const& paramList,
                    xmlrpc_c::value *   const  retvalP) = 0;

        ~XMLRPCMethod()
        {};

    protected:
        std::string name;
};

/**
 * @brief Class implementing an XMLRPC Server
 *
 * The server starts on construction and stops on destruction
 * Lifetime of @c XMLRPCMethod passed to the server is managed by the server
 *
 * @c unique_ptr is used because the underlying xmlrpc_c::serverAbyss
 * only accepts pointers for adding methods.
 * Ownership of the method is transferred to xmlrpc_c::serverAbyss
 * which will handle deletion of the method
 */

class XMLRPCServer
{
    public:
        /**
         * @brief Constructor
         * Server starts on return of constructor
         *
         * @param port Port number for the server
         * @param methods @c vector of @c XMLRPCMethod that defines the methods available on the server
         */
        XMLRPCServer(unsigned int port, std::vector<std::unique_ptr<XMLRPCMethod>> methods);
        /**
         * @brief Destructor
         * Also stops the server
         */
        ~XMLRPCServer();

    private:
        bool is_continue;
        const unsigned int PORT;
        std::thread runner;
        std::vector<std::unique_ptr<XMLRPCMethod>> methods;
        std::unique_ptr<xmlrpc_c::serverAbyss> server;
        void run();
};


#endif

