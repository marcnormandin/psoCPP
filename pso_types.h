#ifndef INC_PSO_TYPES_H
#define INC_PSO_TYPES_H

#include <vector>

namespace ParticleSwarmOptimization {
	typedef double VecCom;
	typedef std::vector<VecCom> Vector;
	typedef Vector Position;
	typedef std::vector<Position> Positions;
	typedef Vector Velocity;
	typedef double Weight;
	typedef size_t ParticleId;
	typedef double Fitness;
	typedef std::vector<Fitness> Fitnesses;

}; // namespace

#endif // #ifndef INC_PSO_PSO_H
