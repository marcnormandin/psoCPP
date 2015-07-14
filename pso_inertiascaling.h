#ifndef INC_PSO_INERTIASCALING_H
#define INC_PSO_INERTIASCALING_H

#include "pso_types.h"

namespace ParticleSwarmOptimization {

// Interface to all interia scaling classes
class InertiaScaling {
public:
	virtual ~InertiaScaling() {}
	virtual Weight weight() const = 0;
};

class NoInertiaScaling : public InertiaScaling {
public:
	NoInertiaScaling(const Weight fixedWeight)
	: mFixedWeight(fixedWeight) {

	}

	virtual Weight weight() const {
		return mFixedWeight;
	}

private:
	Weight mFixedWeight;
};

class LinearInertiaScaling : public InertiaScaling {
public:
	LinearInertiaScaling(const Manager* man, const Weight start, const Weight end)
	: mManager(man), mStart(start), mEnd(end) {
		mSlope = (mEnd - mStart) / (1.0 * mManager->numIterations());
	}

	virtual Weight weight() const {
		Weight w = mSlope * mManager->iteration() + mStart;
		return w;
	}

private:
	const Manager* const mManager;
	Weight mStart;
	Weight mEnd;
	double mSlope;
};

} // namespace ParticleSwarmOptimization

#endif // #ifndef INC_PSO_INERTIASCALING_H
