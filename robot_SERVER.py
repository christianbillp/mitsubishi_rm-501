# This app hosts a python remote control library
from xmlrpc.server import SimpleXMLRPCServer
from xmlrpc.server import SimpleXMLRPCRequestHandler
import numpy as np
import pandas as pd

result_name = 'result.dat'


# Restrict to a particular path.
class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)


# Create server
server = SimpleXMLRPCServer(("deuscortex.com", 4343),
                            requestHandler=RequestHandler)
server.register_introspection_functions()


# Just for testing
def adder_function(x,y):
    return x + y
server.register_function(adder_function, 'add')


# Matrix multiplication
def mmul(filename):
    test_matrix = np.matrix(pd.read_csv(filename))
    result_df = pd.DataFrame(np.matmul(test_matrix, test_matrix))
    result_df.to_csv(result_name, index=False)
    return "Result Ready"
server.register_function(mmul, 'mmul')


# Run the server's main loop
server.serve_forever()

