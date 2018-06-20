#ifndef _WB_SOT_SOLVERS_CBC_BE_H_
#define _WB_SOT_SOLVERS_CBC_BE_H_

#include <OpenSoT/solvers/BackEnd.h>
#include <coin/CbcModel.hpp>
#include <coin/OsiCbcSolverInterface.hpp>
#include <coin/CbcOrClpParam.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Sparse>

namespace OpenSoT{
namespace solvers{
 
    class CBCBackEnd:  public BackEnd{
        public:
            CBCBackEnd(const int number_of_variables,
                    const int number_of_constraints,
                    const double eps_regularisation = 0.0);
            
            ~CBCBackEnd();
            
            /**
             * H NOT USED!
             * g Linear cost function: g'x
             * A Linear Constraints Ax
             * lA/uA constraints
             * l/u bounds
             **/ 
            virtual bool initProblem(const Eigen::MatrixXd &H, const Eigen::VectorXd &g,
                                     const Eigen::MatrixXd &A,
                                     const Eigen::VectorXd &lA, const Eigen::VectorXd &uA,
                                     const Eigen::VectorXd &l, const Eigen::VectorXd &u);
            
            virtual void setOptions(const boost::any& options);
            boost::any getOptions();
            
            struct CBCBackEndOptions
            {
                std::vector<int> integer_ind;
            };
            
        private:
            
            boost::shared_ptr<OsiCbcSolverInterface> _solver;
            boost::shared_ptr<CbcModel> _model;
            
            CoinPackedMatrix _ACP;
            Eigen::SparseMatrix<double> _AS;
            
            CBCBackEndOptions _opt;
            
    };
    
}
}

#endif