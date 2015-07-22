# psoCPP
Particle Swarm Optimization algorithm written in C++ and easily configurable

# Requirements
- C++98 compiler
- GSL - GNU Scientific Library for random number generation. Replace rng.h if you have your own.


# How to extend
- To implement a different topology, inherit from ParticleSwarmOptimization::Topology.
- Inherit from ParticleSwarmOptimization::Manager and implement the virtual member function std::vector<ParticleSwarmOptimization::Fitness> evaluateFunction(const std::vector<ParticleSwarmOptimization::Position>& positions), which should evaluate the function being evaluated and return the function values for the vector of positions given.
