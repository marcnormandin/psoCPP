#ifndef INC_PSO_PARTICLE_H
#define INC_PSO_PARTICLE_H

#include "pso_types.h"

namespace ParticleSwarmOptimization {

class Manager;

class Particle {
	public:
		struct State {
			// !Fixme The magic number is a hack
			State ( const Position& p, const Velocity& v, const Fitness f = -12345 );

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

		const State& best() const;

		const State& current() const;

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
