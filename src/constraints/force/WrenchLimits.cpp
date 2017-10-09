#include <OpenSoT/constraints/force/WrenchLimits.h>

using namespace OpenSoT::constraints::force;

WrenchLimits::WrenchLimits(const double wrenchLimit,
                               const unsigned int x_size) :
    Constraint("wrench_limits", x_size){

    _lowerBound.setZero(getXSize());
    _upperBound.setZero(getXSize());

   this->setWrenchLimits(wrenchLimit);

    this->generateBounds();
}

double WrenchLimits::getWrenchLimits()
{
    return _WrenchLimit;
}

void WrenchLimits::setWrenchLimits(const double wrenchLimit)
{
    _WrenchLimit = std::abs(wrenchLimit);

    this->generateBounds();
}

void WrenchLimits::generateBounds()
{
    /************************ COMPUTING BOUNDS ****************************/
        _lowerBound<<_lowerBound.setOnes(getXSize())*-1.0*_WrenchLimit;
        _upperBound<<_upperBound.setOnes(getXSize())*1.0*_WrenchLimit;
        
        setUpperBound(_upperBound);
        setLowerBound(_lowerBound);

    /**********************************************************************/
}
