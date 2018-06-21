#include <OpenSoT/solvers/CBCBackEnd.h>

using namespace OpenSoT::solvers;

CBCBackEnd::CBCBackEnd(const int number_of_variables, const int number_of_constraints, const double eps_regularisation): 
    BackEnd(number_of_variables, number_of_constraints)
{
    _solver = new OsiCbcSolverInterface;
    
    __generate_data_struct(number_of_variables, number_of_constraints);
    
}
bool CBCBackEnd::solve()
{
    _solver->loadProblem(_ACP, _l.data(), _u.data(), _g.data(), _lA.data(), _uA.data());
    
    _model->assignSolver(_solver, true);
    _model->branchAndBound();
    
    if(!_model->isProvenInfeasible())
        _solution = Eigen::Map<Eigen::VectorXd>(_model->bestSolution(), getNumVariables());
    else
    {
        XBot::Logger::error("CbcModel return unfeasible solution in solve!");
        return false;
    }
    return true;
}


bool CBCBackEnd::initProblem(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, 
                             const Eigen::MatrixXd& A, const Eigen::VectorXd& lA, const Eigen::VectorXd& uA, 
                             const Eigen::VectorXd& l, const Eigen::VectorXd& u)
{
    _H = H; _g = g; _A = A; _lA = lA; _uA = uA; _l = l; _u = u;
    
    if(!(_l.rows() == _u.rows())){
        XBot::Logger::error("l size: %i \n", _l.rows());
        XBot::Logger::error("u size: %i \n", _u.rows());
        assert(_l.rows() == _u.rows());
        return false;}
    if(!(_lA.rows() == _A.rows())){
        XBot::Logger::error("lA size: %i \n", _lA.rows());
        XBot::Logger::error("A rows: %i \n", _A.rows());
        assert(_lA.rows() == _A.rows());
        return false;}
    if(!(_lA.rows() == _uA.rows())){
        XBot::Logger::error("lA size: %i \n", _lA.rows());
        XBot::Logger::error("uA size: %i \n", _uA.rows());
        assert(_lA.rows() == _uA.rows());
        return false;}
        
    _ACP.copyOf(true, _A.rows(), _A.cols(), _AS.nonZeros(), _A.data(), _AS.innerIndexPtr(), _AS.outerIndexPtr(), _AS.innerNonZeroPtr());
    
    _solver->loadProblem(_ACP, _l.data(), _u.data(), _g.data(), _lA.data(), _uA.data());
    _solver->initialSolve();
    

    
    _model.reset(new CbcModel(*_solver));
    _model->setLogLevel(1);
    _model->branchAndBound(); 
    
    
    if(!_model->isProvenInfeasible())
        _solution = Eigen::Map<Eigen::VectorXd>(_model->bestSolution(), getNumVariables());
    else
    {
        XBot::Logger::error("CbcModel return unfeasible solution in initProblem!");
        return false;
    }
    return true;
}

boost::any CBCBackEnd::getOptions()
{
    return _opt;
}


void CBCBackEnd::setOptions(const boost::any& options)
{
    _opt = boost::any_cast<CBCBackEndOptions>(options);
    
    auto it_wrong = std::find_if(_opt.integer_ind.begin(), _opt.integer_ind.end(), [this](int i){ return i >= getNumVariables(); });    
    if(it_wrong != _opt.integer_ind.end())
        XBot::Logger::error("Index of integer variable greater than number of variables! Options will not be applied!");
    else
    {
        if(_opt.integer_ind.size() <= getNumVariables())
            _solver->setInteger(_opt.integer_ind.data(), _opt.integer_ind.size());
        else
             XBot::Logger::error("Size of integer variable greater than number of variables! Options will not be applied!");
    }
}

void CBCBackEnd::__generate_data_struct(const int number_of_variables, 
                                        const int number_of_constraints)
{
    
    /* Set appropriate sparsity pattern to P (upper triangular) */
    
    Eigen::MatrixXd sp_pattern;
    /* Set appropriate sparsity pattern to A (dense + diagonal) */
    sp_pattern.setOnes(number_of_constraints, number_of_variables);
    
    _AS = sp_pattern.sparseView();
    _AS.makeCompressed();

   
    _ACP.copyOf(true, _AS.rows(), _AS.cols(), _AS.nonZeros(), _AS.valuePtr(), _AS.innerIndexPtr(), _AS.outerIndexPtr(), _AS.innerNonZeroPtr());
}

bool CBCBackEnd::updateConstraints(const Eigen::Ref<const Eigen::MatrixXd>& A, 
                                const Eigen::Ref<const Eigen::VectorXd>& lA, 
                                const Eigen::Ref<const Eigen::VectorXd>& uA)
{    
    if(A.rows())
    {
        bool success = BackEnd::updateConstraints(A, lA, uA);

        if(!success)
        {
            return false;
        }
        

        _ACP.copyOf(true, _A.rows(), _A.cols(), _AS.nonZeros(), _A.data(), _AS.innerIndexPtr(), _AS.outerIndexPtr(), _AS.innerNonZeroPtr());
        
    }
    
    return true;
}

CBCBackEnd::~CBCBackEnd()
{

}

double CBCBackEnd::getObjective()
{
   return _model->getObjValue();
}
    
