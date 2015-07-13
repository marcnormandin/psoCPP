# psoCPP
Particle Swarm Optimization algorithm written in C++ and easily configurable

# How to extend

- To implement a different topology, inherit from ParticleSwarmOptimization::Topology.
- Inherit from 
- Inherit from ParticleSwarmOptimization::Manager and implement the virtual member function std::vector<ParticleSwarmOptimization::Fitness> evaluateFunction(const std::vector<ParticleSwarmOptimization::Position>& positions), which should evaluate the function being evaluated and return the function values for the vector of positions given.
