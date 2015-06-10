#ifndef INC_PSO_TOPOLOGY_H
#define INC_PSO_TOPOLOGY_H

#include <vector>

namespace ParticleSwarmOptimization {

	// Interface to particle communication topologies
	class RingTopology {
	public:
		RingTopology() {
			mNumNeighbors = 2;
		}

		virtual void update (const Manager& manager) {
			// Update the social best for all particles using a 
			// moving maximum (in a window)
			// See: http://articles.leetcode.com/2011/01/sliding-window-maximum.html
			// See: http://stackoverflow.com/questions/8905525/computing-a-moving-maximum
		}

		virtual const Position& socialBest (const Particle& asker) {
			// get the neighbor particle ids

			// get the neighbor with the best fitness

			// return the best neighbors best position
		}

	protected:
		size_t numNeighbors() const {
			return mNumNeighbors;
		}

		ParticleId getWrappedId( const ParticleId id, const ParticleId maxId ) {
			if ( (id >= 0) && (id <= maxId) ) {
				// No wrapping
				return id;
			} else if (id < 0) {
				// Wrapping to high values
				return (maxId + id);
			} else {
				// Wrapping to low values
				return (id - maxId);
			}
		}

		std::vector<ParticleId> getNeighborParticleIds (const ParticleId id) {
			std::vector<ParticleId> neighbors;
			for (size_t i = 0; i < numNeighbors(); i++) {
			}


			return neighbors;
		}

	private:
		size_t mNumNeighbors;
		std::vector<Particle> mParticles;
	};
};

#endif // #ifndef INC_PSO_TOPOLOGY_H
