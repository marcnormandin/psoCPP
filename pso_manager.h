#ifndef INC_PSO_MANAGER_H
#define INC_PSO_MANAGER_H

#include <vector>
#include "pso_types.h"

class RandomNumberGenerator;

namespace ParticleSwarmOptimization {

	class Particle;

	class Manager {
		friend class Particle;

	public:
		Manager ( const size_t numDimensions, const size_t numParticles, const size_t maxIterations );

		virtual ~Manager ();

		void estimate ();

	protected:
		void iterate ();

		bool keepLooping();

		size_t iteration() const;
		
		// This should compute a fitness function
		virtual void updateFitnesses () {

		}

		size_t numDimensions () const;

		// Returns the social best position for the given particle
		/*
		const Position& socialBest (const Particle& asker );
		*/

		Weight inertiaWeight() const;

		Weight cognitiveWeight() const;

		Weight socialWeight() const;

		ParticleId genUniqueId ();

		double uniform(const double low = -1, const double high = 1);

		// interface to Particle
		Position randomPosition();
		Velocity randomVelocity();

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

		RandomNumberGenerator* mRng;
	};

}; // namespace

#endif // #ifndef INC_PSO_MANAGER_H
