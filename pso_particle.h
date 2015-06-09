#ifndef INC_PSO_PARTICLE_H
#define INC_PSO_PARTICLE_H

#include "pso_types.h"

namespace ParticleSwarmOptimization {

class Manager;

class Particle {
	public:
		struct State {
			State ( const Position& p, const Velocity& v, const Fitness f = std::numeric_limits<Fitness>::max() );

			Position position;
			Velocity velocity;
			Fitness fitness;
		};

		Particle ( Manager* man, const Particle::State& initialState, const ParticleId id );

		void iterate();

		ParticleId id() const;

		const Position& position() const;

		// Sets the fitness for the current position
		void updateFitness (const Fitness fitness);

	protected:
		void evolveVelocity ();

		void evolvePosition ();

		void applyVelocityConstraint ();

		void applyPositionConstraint ();

		void applyPositionAndVelocityConstraint ();
		
		void updateBest ();

	private:
		Particle (const Particle&);
		void operator=(const Particle&);

		Manager* mManager;
		ParticleId mId;

		State mCurrent;
		State mBest;
	};

}; // namespace

#endif // #ifndef INC_PSO_PARTICLE_H
