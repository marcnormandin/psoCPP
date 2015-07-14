#include <iostream>
#include <stdexcept>
#include <cstdlib>

#include "pso.h"

#include <gsl/gsl_math.h>
#include <gsl/gsl_sf.h>

const size_t NUM_TRIALS = 1000;
const size_t NUM_DIMENSIONS = 2;
const size_t NUM_PARTICLES = 20;
const size_t MAX_ITERATIONS = 10;

const double RANGE = 4.0;
const double TRUE_X = 2.9;
const double TRUE_Y = -0.25;

class AckleyFunction
{
public:
    // Formula found from:
    // http://anvilofcode.wordpress.com/2012/01/21/optimizing-the-ackleys-function/
    AckleyFunction(const double range = 1.0, const double muX = 0.0, const double muY = 0.0)
        : mRange(range), mTrueX(muX), mTrueY(muY)
    {
    }

    bool test() const {
    	if (
    		(convert(-1.0) != -mRange) ||
    		(convert(1.0) != mRange) ||
    		(convert(0.0) != 0.0) ) {
    		return false;
    	}

    	return true;
    }

    double min() const {
    	return -1.0*mRange;
    }

    double max() const {
    	return 1.0*mRange;
    }

    double convert(const double pso_coord) const {
    	return convert(min(), max(), pso_coord);
    }

    // Converts pso coordinates to function coordinates
    static double convert(const double min, const double max, const double pso_coord) {
    	const double func_coord = 0.5*max*(1.0+pso_coord) + 0.5*min*(1.0-pso_coord);

    	//std::cout << pso_coord << " -> " << func_coord << "\n";

    	return func_coord;
    }

    double operator()(const double pso_x, const double pso_y) const
    {
    	const double x = convert(pso_x);
    	const double y = convert(pso_y);

    	if (x < min() || x > max() || y < min() || y > max()) {
    		return std::numeric_limits<ParticleSwarmOptimization::Fitness>::max();
    	}

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
    double mRange;
};


class MyManager : private ParticleSwarmOptimization::Manager {
public:
	MyManager(const gslseed_t seed, const size_t numTrials, const size_t numDimensions, const size_t numParticles, const size_t maxIterations )
	: ParticleSwarmOptimization::Manager(seed, numDimensions, numParticles, maxIterations), mNumTrials(numTrials),
	  mTestFunction(RANGE,TRUE_X, TRUE_Y) {
	  	if(!mTestFunction.test()) {
	  		throw std::runtime_error("Test function failed self-diagnostic test.\n");
	  	}

        mCurrentTrial = 0;
	}

	void performAnalysis() {
        for (mCurrentTrial = 0; mCurrentTrial < mNumTrials; mCurrentTrial++) {
            ParticleSwarmOptimization::Manager::reset();
            ParticleSwarmOptimization::Manager::estimate();
            ParticleSwarmOptimization::Position estimate = getEstimate();
            recordTrialEstimate( estimate );
        }
	}

protected:
    ParticleSwarmOptimization::Position getEstimate() const {
        ParticleSwarmOptimization::Position pos = ParticleSwarmOptimization::Manager::getEstimate();
        pos[0] = mTestFunction.convert(pos[0]);
        pos[1] = mTestFunction.convert(pos[1]);
        return pos;
    }

    size_t trial() const {
        return mCurrentTrial;
    }

	virtual void iterate () {
		ParticleSwarmOptimization::Manager::iterate();

		// Get the best estimate for this iteration step
		ParticleSwarmOptimization::Position estimate = this->getEstimate();

        recordIterationEstimate(estimate);
	}

    void recordTrialEstimate(const ParticleSwarmOptimization::Position& estimate) {
        std::cout << estimate[0] << " " << estimate[1] << std::endl;
    }

    void recordIterationEstimate(const ParticleSwarmOptimization::Position& estimate) {
        //std::cout << "Iteration #" << iteration() << " best is (" << estimate[0] << ", " << estimate[1] << ")\n";
    }

	// The manager calls this function for each interation of the PSO estimation procedure.
	virtual std::vector<ParticleSwarmOptimization::Fitness> evaluateFunction(const std::vector<ParticleSwarmOptimization::Position>& positions) {
		std::vector<ParticleSwarmOptimization::Fitness> fitnesses;
		for (size_t i = 0; i < positions.size(); i++) {
			fitnesses.push_back( f_of_x( positions[i] ) );
		}

		return fitnesses;
	}

    ParticleSwarmOptimization::Fitness f_of_x (const ParticleSwarmOptimization::Position& pos) {
		return mTestFunction(pos[0], pos[1]);
	}

private:
	AckleyFunction mTestFunction;
    size_t mNumTrials;
    size_t mCurrentTrial;
};
	
int main(int argc, char* argv[]) {
	try {
        if (argc != 2) {
            throw std::runtime_error("Invalid arguments");
        }

        size_t arg_numIterations = atoi(argv[1]);

		const gslseed_t seed = 0;
		MyManager man( seed, NUM_TRIALS, NUM_DIMENSIONS, NUM_PARTICLES, arg_numIterations);
		man.performAnalysis();
		//ParticleSwarmOptimization::Position est = man.getEstimate();
		//std::cout << "true x = " << TRUE_X << "\n"; //<< ", and estimated x = " << est[0] << std::endl;
		//std::cout << "true y = " << TRUE_Y << "\n"; //, and estimated y = " << est[1] << std::endl;
	} catch(const std::exception& e) {
		std::cerr << "ERROR: " << e.what() << std::endl;
		return -1;
	}

	return 0;
}
