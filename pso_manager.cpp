#include <functional>

#include "rng.h"

#include "pso_manager.h"

#include "pso_particle.h"

#include "pso_topology.h"

#include <iostream>

#include <cstddef>

namespace ParticleSwarmOptimization {

	class ParticleBestFitnessCmpp {
		public:
			bool operator()(const Particle* const a, const Particle* const b) const {
				const Fitness fita = a->best().fitness;
				const Fitness fitb = b->best().fitness;
				return fita < fitb;
			}
	};


	Manager::Manager ( const gslseed_t seed, const size_t numDimensions, const size_t numParticles, const size_t maxIterations )
	: mNumDimensions(numDimensions), mMaxIterations(maxIterations), mIterationCount(0) {
		mRng = new RandomNumberGenerator( seed );

		mTopology = new RingTopology (this);

		// default settings
		loadStandardWeights();
		setMaxSpeedPerDimension(0.5);
		enableMaxSpeedPerDimension();

		createParticles( numParticles );
	}

	Manager::~Manager () {
		destroyParticles();

		delete mRng;
	}

	void Manager::createParticles(const size_t numParticles) {
		// Initialize the particles
		mParticles.reserve( numParticles );
		for (size_t i = 0; i < numParticles; i++) {
			Particle* p = new Particle ( this, Particle::State(randomPosition(), randomVelocity()), genUniqueId() );
			mParticles.push_back( p );
		}
	}

	void Manager::destroyParticles() {
		// Delete the particles
		while (!mParticles.empty()) {
			delete mParticles.back();
			mParticles.pop_back();
		}
	}

	void Manager::resetParticles() {
		const size_t np = numParticles();
		destroyParticles();
		createParticles(np);
	}

	void Manager::loadStandardWeights() {
		setInertiaWeight(0.72984);
		setSocialWeight(1.496172);
		setCognitiveWeight(1.496172);
	}

	void Manager::estimate () {
		while ( keepLooping() ) {
			iterate ();
		}
	}

	Position Manager::getEstimate() const {
		const Particle* p = *std::min_element(mParticles.begin(), mParticles.end(), ParticleBestFitnessCmpp());
		return p->best().position;
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
		updateParticleFitnesses();

		mIterationCount++;
	}

	bool Manager::keepLooping() {
		return (mIterationCount < mMaxIterations);
	}

	size_t Manager::iteration() const {
		return mIterationCount;
	}

	void Manager::updateParticleFitnesses () {
		std::vector<Position> positions;
		for (int i = 0; i < mParticles.size(); i++) {
			positions.push_back( mParticles[i]->current().position );
		}

		// Call evaluate function for each particle's position, save the fitness
		std::vector<Fitness> fitnesses = evaluateFunction( positions );

		// Update the particle's new fitness value
		for (int i = 0; i < fitnesses.size(); i++) {
			mParticles[i]->updateFitness( fitnesses[i] );
		}
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

	double Manager::maxSpeedPerDimension() const {
		return mMaxSpeedPerDimension;
	}

	void Manager::setMaxSpeedPerDimension(const double newSpeed) {
		//!Fixme make sure value is positive
		mMaxSpeedPerDimension = newSpeed;
	}

	void Manager::enableMaxSpeedPerDimension() {
		mIsEnabledMaxSpeedPerDimension = true;
	}

	void Manager::disableMaxSpeedPerDimension() {
		mIsEnabledMaxSpeedPerDimension = false;
	}

	bool Manager::isEnabledMaxSpeedPerDimension() const {
		return mIsEnabledMaxSpeedPerDimension;
	}


	Weight Manager::inertiaWeight() const {
		return mInertiaWeight;
	}

	void Manager::setInertiaWeight(const Weight newWeight) {
		mInertiaWeight = newWeight;
	}

	Weight Manager::cognitiveWeight() const {
		return mCognitiveWeight;
	}

	void Manager::setCognitiveWeight(const Weight newWeight) {
		mCognitiveWeight = newWeight;
	}

	Weight Manager::socialWeight() const {
		return mSocialWeight;
	}

	void Manager::setSocialWeight(const Weight newWeight) {
		mSocialWeight = newWeight;
	}

	double Manager::uniform(const double low, const double high) {
		return mRng->uniform(low, high);
	}

	ParticleId Manager::genUniqueId () {
		return mParticles.size();
	}

}; // namespace
