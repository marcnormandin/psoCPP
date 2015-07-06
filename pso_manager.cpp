#include <functional>

#include "rng.h"

#include "pso_manager.h"

#include "pso_particle.h"

#include "pso_topology.h"

#include <iostream>

#include <cstddef>

namespace ParticleSwarmOptimization {

	Manager::Manager ( const size_t numDimensions, const size_t numParticles, const size_t maxIterations )
	: mNumDimensions(numDimensions), mMaxIterations(maxIterations), mIterationCount(0) {
		// Init the RNG
		const gslseed_t seed = 0;
		mRng = new RandomNumberGenerator( seed );

		mTopology = new RingTopology (this);

		// Initialize the particles
		mParticles.reserve( numParticles );
		for (size_t i = 0; i < numParticles; i++) {
			Particle* p = new Particle ( this, Particle::State(randomPosition(), randomVelocity()), genUniqueId() );
			mParticles.push_back( p );
		}
	}

	Manager::~Manager () {
		// Delete the particles
		while (!mParticles.empty()) {
			delete mParticles.back();
			mParticles.pop_back();
		}

		delete mRng;
	}

	void Manager::estimate () {
		while ( keepLooping() ) {
			std::cout << "Manager::Iteration #" << iteration() << " ...\n\n";
			iterate ();
		}
	}

	size_t Manager::numParticles() const {
		return mParticles.size();
	}
	
	const Particle& Manager::particle(const ParticleId pid) const {
		return *mParticles.at(pid);
	}

	void Manager::iterate () {
		// Update the topology
		mTopology->update();
		
		// iterate each particle
		std::for_each(mParticles.begin(), mParticles.end(), std::mem_fun(&Particle::iterate));

		// Update each particle's fitness
		updateFitnesses();

		mIterationCount++;
	}

	bool Manager::keepLooping() {
		return (mIterationCount < mMaxIterations);
	}

	size_t Manager::iteration() const {
		return mIterationCount;
	}

	void Manager::updateFitnesses () {
		// !ToDo
	}

	size_t Manager::numDimensions () const {
		return mNumDimensions;
	}

	// Returns the social best position for the given particle
	const Position& Manager::socialBest (const Particle& asker) {
		// Depending on the topology, return the particles social best
		return mTopology->socialBest( asker );
	}

	Position Manager::randomPosition() {
		Position pos( numDimensions() );
		for (size_t d = 0; d < pos.size(); d++) {
			pos[d] = uniform();
		}
		return pos;
	}

	Velocity Manager::randomVelocity() {
		Velocity vel( numDimensions() );
		for (size_t d = 0; d < vel.size(); d++) {
			vel[d] = uniform();
		}
		return vel;
	}

	Weight Manager::inertiaWeight() const {
		return mInertiaWeight;
	}

	Weight Manager::cognitiveWeight() const {
		return mCognitiveWeight;
	}

	Weight Manager::socialWeight() const {
		return mSocialWeight;
	}

	double Manager::uniform(const double low, const double high) {
		return mRng->uniform(low, high);
	}

	ParticleId Manager::genUniqueId () {
		return mParticles.size();
	}

}; // namespace
