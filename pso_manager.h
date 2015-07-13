#ifndef INC_PSO_MANAGER_H
#define INC_PSO_MANAGER_H

#include <vector>
#include "pso_types.h"

class RandomNumberGenerator;

// Seeds for GNU GSL random number generators
// This is defined in rng.h
typedef unsigned long int gslseed_t;

namespace ParticleSwarmOptimization {

	class Particle;
	class Topology;

	class Manager {
		friend class Particle;

	public:
		Manager (const gslseed_t seed, const size_t numDimensions, const size_t numParticles, const size_t maxIterations );

		virtual ~Manager ();

		void loadStandardWeights();

		void estimate ();

		Position getEstimate() const;

		size_t numParticles() const;

		const Particle& particle(const ParticleId pid) const;

		double maxSpeedPerDimension() const;
		void setMaxSpeedPerDimension(const double newSpeed);
		void enableMaxSpeedPerDimension();
		void disableMaxSpeedPerDimension();
		bool isEnabledMaxSpeedPerDimension() const;

	protected:
		virtual void iterate ();

		bool keepLooping();

		size_t iteration() const;
		
		// This should evaluate a fitness function, e.g. z = f(x,y)
		virtual std::vector<Fitness> evaluateFunction (const std::vector<Position>& positions ) = 0;

		void updateParticleFitnesses ();

		size_t numDimensions () const;

		// Returns the social best position for the given particle
		const Position& socialBest (const Particle& asker );

		Weight inertiaWeight() const;
		void setInertiaWeight(const Weight newWeight);

		Weight cognitiveWeight() const;
		void setCognitiveWeight(const Weight newWeight);

		Weight socialWeight() const;
		void setSocialWeight(const Weight newWeight);

		ParticleId genUniqueId ();

		double uniform(const double low = -1, const double high = 1);

		// interface to Particle
		Position 	randomPosition();
		Velocity 	randomVelocity();

		void createParticles(const size_t numParticles);
		void destroyParticles();

	private:
		Manager (const Manager&);
		void operator=(const Manager&);
		
		size_t mNumDimensions;
		size_t mMaxIterations;
		std::vector<Particle*> mParticles;

		Weight mInertiaWeight;
		Weight mSocialWeight;
		Weight mCognitiveWeight;

		size_t mIterationCount;

		double mMaxSpeedPerDimension;
		bool mIsEnabledMaxSpeedPerDimension;

		Topology* mTopology;

		RandomNumberGenerator* mRng;
	};

}; // namespace

#endif // #ifndef INC_PSO_MANAGER_H
