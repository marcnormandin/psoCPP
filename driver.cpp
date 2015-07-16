#include <iostream>
#include <stdexcept>
#include <cstdlib>

#include "pso.h"

#include <gsl/gsl_math.h>
#include <gsl/gsl_sf.h>

const size_t NUM_TRIALS = 10;
const size_t NUM_DIMENSIONS = 2;
const size_t NUM_PARTICLES = 20;
const size_t MAX_ITERATIONS = 100;

const double RANGE = 4.0;
const double TRUE_X = 2.9;
const double TRUE_Y = -0.25;

// Converts from pso coordinates to function coordinates
// Specifically: PSO's [-1, 1] to  Func's [min, max]
double convertPSOCoord(const double min, const double max, const double pso_coord) {
    const double func_coord = 0.5*max*(1.0+pso_coord) + 0.5*min*(1.0-pso_coord);

    return func_coord;
}

// The function whose minimum is to be estimated
class AckleyFunction
{
public:
    // Formula found from:
    // http://anvilofcode.wordpress.com/2012/01/21/optimizing-the-ackleys-function/
    AckleyFunction(const double range = 1.0, const double muX = 0.0, const double muY = 0.0)
        : mRange(range), mTrueX(muX), mTrueY(muY)
    {
        if(!test()) {
            throw std::runtime_error("Test function failed self-diagnostic test.\n");
        }
    }

    // Called by PRDAnalysis
    ParticleSwarmOptimization::Position convertCoords(const ParticleSwarmOptimization::Position& psoCoord) const {
        ParticleSwarmOptimization::Position funcCoord;
        for (int i = 0; i < psoCoord.size(); i++) {
            funcCoord.push_back( convert(psoCoord[i]) );
        }
        return funcCoord;
    }

    // Called by PRDAnalysis
    ParticleSwarmOptimization::Fitness operator()(const ParticleSwarmOptimization::Position& pos) const {
        if (pos.size() != 2) {
            throw std::runtime_error("ERROR: AckelyFunction requires only 2 coordinates");
        }

        return eval(pos[0], pos[1]);
    }

protected:
    // The actual function
    double eval(const double pso_x, const double pso_y) const
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


    double min() const {
        return -1.0*mRange;
    }

    double max() const {
        return 1.0*mRange;
    }

    double convert(const double pso_coord) const {
        return convertPSOCoord(min(), max(), pso_coord);
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

private:
    double mTrueX;
    double mTrueY;
    double mRange;
};



// Performs the PRD analysis.
// Runs many trials to obtain statistics for one model system
template<typename FitnessFunction>
class PRDAnalysis : private ParticleSwarmOptimization::Manager {
public:
	PRDAnalysis(FitnessFunction& ff, const gslseed_t seed, const size_t numPSOTrials, const size_t numDimensions, const size_t numParticles, const size_t maxIterations )
	: ParticleSwarmOptimization::Manager(seed, numDimensions, numParticles, maxIterations), 
      mFitnessFunction(ff), mNumPSOTrials(numPSOTrials) {
        mCurrentPSOTrial = 0;
	}

	void perform() {
        for (mCurrentPSOTrial = 0; mCurrentPSOTrial < mNumPSOTrials; mCurrentPSOTrial++) {
            performPSOTrial();
        }
	}

protected:
    void performPSOTrial() {
        // Perform a trial
        ParticleSwarmOptimization::Manager::reset();
        ParticleSwarmOptimization::Manager::estimate();
        ParticleSwarmOptimization::Position estimate = getEstimate();
        recordTrialEstimate( estimate );
    }

    ParticleSwarmOptimization::Position getEstimate() const {
        ParticleSwarmOptimization::Position pos = ParticleSwarmOptimization::Manager::getEstimate();
        // Convert from PSO coordinates to Function coordinates
        pos = mFitnessFunction.convertCoords(pos);
        return pos;
    }

    size_t trial() const {
        return mCurrentPSOTrial;
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
			fitnesses.push_back( mFitnessFunction( positions[i] ) );
		}

		return fitnesses;
	}

private:
	FitnessFunction mFitnessFunction;
    size_t mNumPSOTrials;
    size_t mCurrentPSOTrial;
};
	
int main(int argc, char* argv[]) {
	try {
        if (argc != 2) {
            throw std::runtime_error("Invalid arguments");
        }

        size_t arg_numIterations = atoi(argv[1]);

        AckleyFunction ff(RANGE,TRUE_X, TRUE_Y);

		const gslseed_t seed = 0;
		PRDAnalysis<AckleyFunction> analysis(ff, seed, NUM_TRIALS, NUM_DIMENSIONS, NUM_PARTICLES, arg_numIterations);

		analysis.perform();
		std::cout << "true: " << TRUE_X << " " << TRUE_Y << "\n";
	} catch(const std::exception& e) {
		std::cerr << "ERROR: " << e.what() << std::endl;
		return -1;
	}

	return 0;
}
