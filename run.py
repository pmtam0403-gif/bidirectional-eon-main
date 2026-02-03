import time
from src.Simulator import Simulator

if __name__ == '__main__':
    forced_load = 50
    # num_simulations = 1
    #
    # Simulator(sim_config_file, trace, verbose, forced_load, num_simulations)
    trace=Simulator.trace 
    verbose=Simulator.verbose 
    min_load = 10
    max_load = 200
    step = 20
    num_simulations = 1
    nfs="D:/bidirectional-eon-main/xml/nfs.xml" 


    for forced_load in range(min_load, max_load + 1, step):

        sim=Simulator(nfs, trace, verbose, forced_load, num_simulations)
 