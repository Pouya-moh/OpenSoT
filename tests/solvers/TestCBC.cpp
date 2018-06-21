#include <gtest/gtest.h>
#include <OpenSoT/solvers/CBCBackEnd.h>
#include <OpenSoT/tasks/GenericTask.h>
#include <OpenSoT/constraints/GenericConstraint.h>

namespace {

class testCBCProblem: public ::testing::Test
{
protected:

    testCBCProblem()
    {

    }

    virtual ~testCBCProblem() {

    }

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

using namespace OpenSoT::tasks;
using namespace OpenSoT::constraints;

TEST_F(testCBCProblem, testMILPProblem)
{
    Eigen::MatrixXd A(3,3); A.setZero(3,3);
    Eigen::VectorXd b(3); b.setZero(3);
    GenericTask::Ptr task(new GenericTask("task",A,b));
    Eigen::VectorXd c(3);
    c<<-3.,-2.,-1;
    //task->
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
