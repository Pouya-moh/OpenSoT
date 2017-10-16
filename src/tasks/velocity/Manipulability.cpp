#include <OpenSoT/tasks/velocity/Manipulability.h>
#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::velocity;

Manipulability::Manipulability(const Eigen::VectorXd& x, const XBot::ModelInterface& robot_model,
                               const Cartesian::Ptr CartesianTask):
    Task("manipulability::"+CartesianTask->getTaskID(), x.size()),
    _manipulabilityIndexGradientWorker(x, robot_model, CartesianTask),
    _x(x)
{
    double _x_size = this->getXSize();

    _W.resize(_x_size, _x_size);
    _W.setIdentity(_x_size, _x_size);
    this->setW(_W);

    this->setHessianType(HST_POSDEF);

    _A.resize(_x_size, _x_size);
    _A.setIdentity(_x_size, _x_size);
    this->setA(_A);

    /* first update. Setting desired pose equal to the actual pose */
    this->_update(x);

}

Manipulability::Manipulability(const Eigen::VectorXd& x, const XBot::ModelInterface& robot_model,
                               const CoM::Ptr CartesianTask):
    Task("manipulability::"+CartesianTask->getTaskID(), x.size()),
    _manipulabilityIndexGradientWorker(x, robot_model,CartesianTask),
    _x(x)
{
    double _x_size = this->getXSize();
    _W.resize(_x_size, _x_size);
    _W.setIdentity(_x_size, _x_size);
    this->setW(_W);

    this->setHessianType(HST_POSDEF);

    _A.resize(_x_size, _x_size);
    _A.setIdentity(_x_size, _x_size);
    this->setA(_A);


    /* first update. Setting desired pose equal to the actual pose */
    this->_update(x);
}

Manipulability::~Manipulability()
{

}

void Manipulability::_update(const Eigen::VectorXd &x) {

    _x = x;
    /************************* COMPUTING TASK *****************************/

    _b = this->getLambda()*cartesian_utils::computeGradient(x, _manipulabilityIndexGradientWorker, this->getActiveJointsMask());
    this->setb(_b);
    /**********************************************************************/
}

double Manipulability::ComputeManipulabilityIndex()
{
    return _manipulabilityIndexGradientWorker.compute(_x);
}
