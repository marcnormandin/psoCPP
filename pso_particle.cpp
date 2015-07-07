#include "pso_particle.h"
#include "pso_manager.h"
#include <limits>
#include <iostream> 
#include <cmath>

namespace ParticleSwarmOptimization {

	Particle::State::State( const Position& p, const Velocity& v, const Fitness f )
	: position(p), velocity(v), fitness( f ) {

	}

	Particle::Particle ( Manager* man, const Particle::State& initialState, const ParticleId id )
	: mManager (man), mId (id), mCurrent (initialState), mBest (initialState) {
	}

	void Particle::iterate() {
		// update velocity
		evolveVelocity();

		// update position
		evolvePosition ();

		// apply combined constraint
		applyPositionAndVelocityConstraint();

		//std::cout << "I'm #" << id() << " and I was just iterated! SWEET!\n";
	}

	void Particle::evolveVelocity () {
		const Position& socialBest = mManager->socialBest( *this );
		for (Velocity::size_type d = 0; d < mCurrent.velocity.size(); ++d) {
			// inertia term
			const VecCom vInertia = mManager->inertiaWeight() * mCurrent.velocity[d];

			// social term
			const double u1 = mManager->uniform(0,1);
			const VecCom vSocial = mManager->socialWeight() * u1 * ( socialBest[d] - mCurrent.position[d] );

			// cognitive term
			const double u2 = mManager->uniform(0,1);
			const VecCom vCognitive = mManager->cognitiveWeight() * u2 * ( mBest.position[d] - mCurrent.position[d] );

			mCurrent.velocity[d] = ( vInertia + vSocial + vCognitive );
		}		
		applyVelocityConstraint ();
	}

	void Particle::evolvePosition () {
		for (Position::size_type d = 0; d < mCurrent.position.size(); ++d) {
			mCurrent.position[d] = ( mCurrent.position[d] + mCurrent.velocity[d] );
		}

		applyPositionConstraint ();
	}

	void Particle::applyVelocityConstraint () {
		const double MAX_DIM_SPEED = 0.5;

		for (size_t i = 0; i < mCurrent.velocity.size(); i++) {
			if (std::fabs(mCurrent.velocity[i]) > MAX_DIM_SPEED) {
				if (mCurrent.velocity[i] < 0) {
					mCurrent.velocity[i] = -1.0 * MAX_DIM_SPEED;
				} else {
					mCurrent.velocity[i] = MAX_DIM_SPEED;
				}
			}
		}
	}

	void Particle::applyPositionConstraint () {
		// !ToDo
	}

	void Particle::applyPositionAndVelocityConstraint () {
		// !ToDo
	}

	bool isPositionWithinBounds(const Position& pos) {
		for (size_t i = 0; i < pos.size(); i++) {
			if ( (pos[i] < -1) || (pos[i] > 1) ) {
				return false;
			}
		}

		return true;
	}

	// Sets the fitness for the current position
	// The manager calls this
	void Particle::updateFitness (const Fitness fitness) {
		mCurrent.fitness = fitness;

		// !Fixme
		// We are getting rid of the evaluation if outside the bounds
		if (!isPositionWithinBounds(current().position)) {
			// Set it to the worst possible value
			mCurrent.fitness = std::numeric_limits<Fitness>::max();
		}

		updateBest ();
	}

	const Particle::State& Particle::best() const {
		return mBest;
	}

	const Particle::State& Particle::current() const {
		return mCurrent;
	}

	void Particle::updateBest () {
		if (mCurrent.fitness < mBest.fitness) {
			mBest = mCurrent;
		}
	}

	ParticleId Particle::id() const {
		return mId;
	}

	const Position& Particle::position() const {
		return mCurrent.position;
	}

}; // namespace
