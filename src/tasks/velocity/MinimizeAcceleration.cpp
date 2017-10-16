#include <OpenSoT/tasks/velocity/MinimizeAcceleration.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;

MinimizeAcceleration::MinimizeAcceleration(const Eigen::VectorXd &x):
    Task("MinimizeAcceleration", x.rows()),
    _x_before(x.rows())
{
    _x_before.setZero(x.rows());

    double _x_size = getXSize();

    _A.resize(_x_size, _x_size);
    _A.setIdentity(_x_size, _x_size);
    setA(_A);

    _W.resize(_x_size, _x_size);
    _W.setIdentity(_x_size, _x_size);
    setW(_W);

    setHessianType(HST_IDENTITY);

    this->_update(x);
}

MinimizeAcceleration::~MinimizeAcceleration()
{

}

void MinimizeAcceleration::_update(const Eigen::VectorXd &x) {
    _b = x - _x_before;
    setb(_b);
    _x_before = x;
}

