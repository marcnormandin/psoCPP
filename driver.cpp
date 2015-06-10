#include "pso.h"

const size_t NUM_DIMENSIONS = 2;
const size_t NUM_PARTICLES = 30;
const size_t MAX_ITERATIONS = 100;

// Driver
int main(int argc, char * argv[]) {
	
	ParticleSwarmOptimization::Manager man( NUM_DIMENSIONS, NUM_PARTICLES, MAX_ITERATIONS);
	man.estimate();

	return 0;
}

