all:
	g++ -o test pso_manager.cpp pso_particle.cpp driver.cpp -I${DEVTOOLS}/include -L${DEVTOOLS}/lib -lgsl -lgslcblas -lm
