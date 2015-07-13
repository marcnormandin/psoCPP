#include <iostream>

#include "pso.h"

#include <gsl/gsl_math.h>
#include <gsl/gsl_sf.h>

const size_t NUM_DIMENSIONS = 2;
const size_t NUM_PARTICLES = 20;
const size_t MAX_ITERATIONS = 100;

const double TRUE_X = -0.1;
const double TRUE_Y = -0.25;

class AckleyFunction
{
public:
    // Formula found from:
    // http://anvilofcode.wordpress.com/2012/01/21/optimizing-the-ackleys-function/
    AckleyFunction(const double muX = 0.0, const double muY = 0.0)
        : mTrueX(muX), mTrueY(muY)
    {
    }

    double operator()(const double x, const double y) const
    {
        const double termOne = 20.0 + gsl_sf_exp(1.0);

        const double termTwoArg = -0.5 * (gsl_pow_2(x - mTrueX) + gsl_pow_2(y - mTrueY));
        const double termTwo = -20.0 * gsl_sf_exp( termTwoArg );

        const double termThreeArg = 0.5 * (gsl_sf_cos(2.0*M_PI*(x - mTrueX)) + gsl_sf_cos(2.0*M_PI*(y - mTrueY)));
        const double termThree = -1.0 * gsl_sf_exp( termThreeArg );

        return (termOne + termTwo + termThree);
    }

private:
    double mTrueX;
    double mTrueY;
};


class MyManager : public ParticleSwarmOptimization::Manager {
public:
	MyManager(const gslseed_t seed, const size_t numDimensions, const size_t numParticles, const size_t maxIterations )
	: ParticleSwarmOptimization::Manager(seed, numDimensions, numParticles, maxIterations),
	  mTestFunction(TRUE_X, TRUE_Y) {

	}

protected:
	// The manager calls this function for each interation of the PSO estimation procedure.
	virtual std::vector<ParticleSwarmOptimization::Fitness> evaluateFunction(const std::vector<ParticleSwarmOptimization::Position>& positions) {
		std::vector<ParticleSwarmOptimization::Fitness> fitnesses;
		for (size_t i = 0; i < positions.size(); i++) {
			fitnesses.push_back( evaluateTestFunction( positions[i] ) );
		}

		return fitnesses;
	}

    ParticleSwarmOptimization::Fitness evaluateTestFunction (const ParticleSwarmOptimization::Position& pos) {
		return mTestFunction(pos[0], pos[1]);
	}

private:
	AckleyFunction mTestFunction;
};

// Driver
int main(int argc, char * argv[]) {
	
	const gslseed_t seed = 0;
	MyManager man( seed, NUM_DIMENSIONS, NUM_PARTICLES, MAX_ITERATIONS);
	man.estimate();
	ParticleSwarmOptimization::Position est = man.getEstimate();
	std::cout << "true x = " << TRUE_X << ", and estimated x = " << est[0] << std::endl;
	std::cout << "true y = " << TRUE_Y << ", and estimated y = " << est[1] << std::endl;

	return 0;
}
