#ifndef INC_PSO_TOPOLOGY_H
#define INC_PSO_TOPOLOGY_H

#include <vector>

#include "pso_particle.h"
#include "pso_manager.h"

namespace ParticleSwarmOptimization {

	class Topology {
	public:
		Topology (const Manager* const manager)
		: mManager(manager) {}

		virtual void update () = 0;
		virtual const Position& socialBest (const Particle& asker) = 0;

	protected:
		const Manager* const manager() const {
			return mManager;
		}

	private:
		const Manager* const mManager;
	};

	class ParticleFitnessCmp {
	public:
		ParticleFitnessCmp(const Manager* const manager) :
		mManager(manager) {}

		bool operator()(const ParticleId a, const ParticleId b) const {
			const Fitness fita = mManager->particle(a).best().fitness;
			const Fitness fitb = mManager->particle(b).best().fitness;
			return fita < fitb;
		}

	private:
		const Manager* const mManager;
	};

	// Interface to particle communication topologies
	class RingTopology : public Topology {
	public:
		RingTopology (const Manager* const manager)
		: Topology(manager) {
			mNumNeighbors = 2;
		}

		virtual void update () {
			// Update the social best for all particles using a 
			// moving maximum (in a window)
			// See: http://articles.leetcode.com/2011/01/sliding-window-maximum.html
			// See: http://stackoverflow.com/questions/8905525/computing-a-moving-maximum

			// This allows us to do the bulk computation at one time
		}

		virtual const Position& socialBest (const Particle& asker) {
			// get the neighbor particle ids
			std::vector<ParticleId> neighbors = getNeighborParticleIds( asker.id() );

			// get the neighbor with the best fitness
			const ParticleId pid = *std::max_element(neighbors.begin(), neighbors.end(), ParticleFitnessCmp(manager()));

			// return the best neighbors best position
			return manager()->particle(pid).best().position;
		}

	protected:
		size_t numNeighbors() const {
			return mNumNeighbors;
		}

		// Returns a valid particle id. Wraps around 0 and max bound.
		ParticleId getWrappedId( const int id ) {
			const size_t maxId = manager()->numParticles() - 1;

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

		std::vector<ParticleId> getNeighborParticleIds (const ParticleId pid) {
			std::vector<ParticleId> neighbors;
			neighbors.reserve( numNeighbors() );

			for (size_t i = 0; i < numNeighbors(); i++) {
				const ParticleId nid = getWrappedId( pid + i );
				neighbors.push_back( nid );
			}

			return neighbors;
		}

	private:
		size_t mNumNeighbors;
	};
};

#endif // #ifndef INC_PSO_TOPOLOGY_H
