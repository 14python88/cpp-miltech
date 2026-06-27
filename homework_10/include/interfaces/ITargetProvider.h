#pragma once
#include <Structs.h>

class ITargetProvider {
public:
    virtual ~ITargetProvider() = default;

    virtual bool isThreadReady() const = 0;
    virtual void start() = 0;
    virtual void stop() = 0;

    virtual std::vector<Target> getTargetNow()       const = 0;
    virtual std::vector<Coord>  getTargetPredicted() const = 0;
    virtual void extrapolateTargets(const std::vector<Params>& params) = 0;

    virtual int getTargetCount() const = 0;
};
 
