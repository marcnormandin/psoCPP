#ifndef INC_PSO_TYPES_H
#define INC_PSO_TYPES_H

#include <vector>

namespace ParticleSwarmOptimization {
	typedef double VecCom;
	typedef std::vector<VecCom> Vector;
	typedef Vector Position;
	typedef Vector Velocity;
	typedef double Weight;
	typedef unsigned int ParticleId;
	typedef double Fitness;
}; // namespace

#endif // #ifndef INC_PSO_PSO_H
