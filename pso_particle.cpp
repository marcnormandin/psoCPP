#include "pso_particle.h"
#include "pso_manager.h"
#include <limits>
#include <iostream> 

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

		std::cout << "I'm #" << id() << " and I was just iterated! SWEET!\n";
	}

	void Particle::evolveVelocity () {
		//const Position& socialBest = mManager->socialBest( *this );	
		for (Velocity::size_type d = 0; d < mCurrent.velocity.size(); ++d) {
			// inertia term
			VecCom vInertia = mManager->inertiaWeight() * mCurrent.velocity[d];

			// social term
			// !Fixme
			/*
			double u1 = mManager->uniform(0,1);
			VecCom vSocial = mManager->socialWeight() * u1 * ( socialBest[d] - mPosition[d] );
			*/
			VecCom vSocial = 0.0;

			// cognitive term
			double u2 = mManager->uniform(0,1);
			VecCom vCognitive = mManager->cognitiveWeight() * u2 * ( mBest.position[d] - mCurrent.position[d] );

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

	}

	void Particle::applyPositionConstraint () {

	}

	void Particle::applyPositionAndVelocityConstraint () {

	}

	// Sets the fitness for the current position
	// The manager calls this
	void Particle::updateFitness (const Fitness fitness) {
		mCurrent.fitness = fitness;

		updateBest ();
	}

	void Particle::updateBest () {
		if (mCurrent.fitness > mBest.fitness) {
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
