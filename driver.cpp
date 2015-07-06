#include "pso.h"

const size_t NUM_DIMENSIONS = 1;
const size_t NUM_PARTICLES = 2;
const size_t MAX_ITERATIONS = 2;

// Driver
int main(int argc, char * argv[]) {
	
	ParticleSwarmOptimization::Manager man( NUM_DIMENSIONS, NUM_PARTICLES, MAX_ITERATIONS);
	man.estimate();

	return 0;
}

