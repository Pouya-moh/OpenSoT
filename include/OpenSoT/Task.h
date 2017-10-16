/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi
 * email:  alessio.rocchi@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __TASK_H__
#define __TASK_H__

 #include <list>
 #include <string>
 #include <vector>
 #include <OpenSoT/Constraint.h>
 #include <assert.h>
 #include <boost/shared_ptr.hpp>
 #include <XBotInterface/Logger.hpp>

 namespace OpenSoT {

    /** Summarises all possible types of the QP's Hessian matrix. From qpOASES/Types.hpp */
    enum HessianType
    {
        HST_ZERO,                   /**< Hessian is zero matrix (i.e. LP formulation). */
        HST_IDENTITY,               /**< Hessian is identity matrix. */
        HST_POSDEF,                 /**< Hessian is (strictly) positive definite. */
        HST_POSDEF_NULLSPACE,       /**< Hessian is positive definite on null space of active bounds/constraints. */
        HST_SEMIDEF,                /**< Hessian is positive semi-definite. */
        HST_UNKNOWN                 /**< Hessian type is unknown. */
    };

    /**
     * @brief Task represents a task in the form \f$T(A,b)\f$ where \f$A\f$ is the task error jacobian and \f$b\f$ is the task error
    */
    class Task {

    public:
        
        typedef boost::shared_ptr<Task> TaskPtr;
        typedef boost::shared_ptr<Constraint> ConstraintPtr;
        
        /**
         * @brief Task define a task in terms of Ax = b
         * @param task_id is a unique id
         * @param x_size is the number of variables of the task
         */
        Task(const std::string task_id,
             const unsigned int x_size) :
            __task_id(task_id), __x_size(x_size), __active_joints_mask(x_size), __is_active(true)
        {
            __lambda = 1.0;
            __hessianType = HST_UNKNOWN;
            for(unsigned int i = 0; i < x_size; ++i)
                __active_joints_mask[i] = true;
        }

        virtual ~Task(){}

        /**
         * @brief Activated / deactivates the task by setting the A matrix to zero.
         * Important note: after activating a task, call the update() function in 
         * order to recompute a proper A matrix.
         */
        void setActive(const bool active_flag){
            
            if(!__is_active && active_flag){
                __A = __A_last_active;
            }
            
            __is_active = active_flag;
        }
        
        
        /**
         * @brief Returns a boolean which specifies if the task is active.
         */
        bool isActive() const {
            return __is_active;
        }
        
        /**
         * @brief getA
         * @return the A matrix of the task
         */
        const Eigen::MatrixXd& getA() const {
            return __A;
        }

        /**
         * @brief getHessianAtype
         * @return the Hessian type
         */
        const HessianType getHessianAtype() { return __hessianType; }

        /**
         * @brief getb
         * @return the b matrix of the task
         */
        const Eigen::VectorXd& getb() const { return __b; }

        /**
         * @brief getWA
         * @return the product between W and A
         */
        const Eigen::MatrixXd& getWA() const {
            __WA.noalias() = __W*__A;
            return __WA;
        }

        /**
         * @brief getATranspose()
         * @return A transposed
         */
        const Eigen::MatrixXd& getATranspose() const {
            __Atranspose = __A.transpose(); //This brakes the use of the template!
            return __Atranspose;
        }

        /**
         * @brief getWb
         * @return the product between W and b
         */
        const Eigen::VectorXd& getWb() const {
            __Wb = __W*__b;
            return __Wb;
        }

        /**
         * @brief getWeight
         * @return the weight of the norm of the task error
         */
        const Eigen::MatrixXd& getWeight() const { return __W; }

        /**
         * @brief setWeight sets the task weight.
         * Note the Weight needs to be positive definite.
         * If your original intent was to get a subtask
         * (i.e., reduce the number of rows of the task Jacobian),
         * please use the class SubTask
         * @param W matrix weight
         */
        virtual void setWeight(const Eigen::MatrixXd& W) {
            assert(W.rows() == this->getTaskSize());
            assert(W.cols() == W.rows());
            __W = W;
        }

        /**
         * @brief getLambda
         * @return the lambda weight of the task
         */
        const double getLambda() const { return __lambda; }

        virtual void setLambda(double lambda)
        {
            if(lambda >= 0.0){
                __lambda = lambda;
            }
        }
        
        
        /**
         * @brief getConstraints return a reference to the constraint list. Use the standard list methods
         * to add, remove, clear, ... the constraints list.
         * e.g.:
         *              task.getConstraints().push_back(new_constraint)
         * @return
         */
        virtual std::list< ConstraintPtr >& getConstraints() { return __constraints; }

        /** Gets the number of variables for the task.
            @return the number of columns of A */
        const unsigned int getXSize() const { return __x_size; }

        /** Gets the task size.
            @return the number of rows of A */
        virtual const unsigned int getTaskSize() const { return __A.rows(); }

        /** Updates the A, b, Aeq, beq, Aineq, b*Bound matrices 
            @param x variable state at the current step (input) */
        void update(const Eigen::VectorXd &x) {
           
            
            for(typename std::list< ConstraintPtr >::iterator i = this->getConstraints().begin();
                i != this->getConstraints().end(); ++i) (*i)->update(x);
            this->_update(x);
            
            if(!__is_active){
                __A_last_active = __A;
                __A.setZero(__A.rows(), __A.cols());
                return;
            }

            typedef std::vector<bool>::const_iterator it_m;
            bool all_true = true;
            for( it_m active_joint = __active_joints_mask.begin();
                 active_joint != __active_joints_mask.end();
                 ++active_joint)
            {
                if(*active_joint == false) all_true = false;
            }

            if(!all_true) applyActiveJointsMask(__A);
            
            
        }

        /**
         * @brief getTaskID return the task id
         * @return a string with the task id
         */
        std::string getTaskID(){ return __task_id; }

        /**
         * @brief getActiveJointsMask return a vector of length NumberOfDOFs.
         * If an element is false the corresponding column of the task jacobian is set to 0.
         * @return a vector of bool
         */
        virtual std::vector<bool> getActiveJointsMask(){return __active_joints_mask;}

        /**
         * @brief setActiveJointsMask set a mask on the Jacobian. The changes take effect immediately.
         * @param active_joints_mask
         * @return true if success
         */
        virtual bool setActiveJointsMask(const std::vector<bool>& active_joints_mask)
        {
            if(active_joints_mask.size() == __active_joints_mask.size())
            {
                __active_joints_mask = active_joints_mask;

                applyActiveJointsMask(__A);

                return true;
            }
            return false;
        }

        /**
         * @brief log logs common Task internal variables
         * @param logger a shared pointer to a MathLogger
         */
        virtual void log(XBot::MatLogger::Ptr logger)
        {
            logger->add(__task_id + "_A", __A);
            logger->add(__task_id + "_b", __b);
            logger->add(__task_id + "_W", __W);
            logger->add(__task_id + "_lambda", __lambda);
            _log(logger);

            for(auto constraint : __constraints)
                constraint->log(logger);

        }
        
        
    protected:
        
        bool setA(const Eigen::MatrixXd& A)
        {
            if(A.cols() != __x_size)
                return false;
            __A = A;
            return true;
        }

        bool setb(const Eigen::VectorXd& b)
        {
            __b = b;
            return true;
        }

        void setHessianType(HessianType hessian_type)
        {
            __hessianType = hessian_type;
        }

        bool setW(const Eigen::MatrixXd& W)
        {
            __W = W;
            return true;
        }


        
        
    private:

        /**
         * @brief _task_id unique name of the task
         */
        std::string __task_id;

        /**
         * @brief _x_size size of the controlled variables
         */
        unsigned int __x_size;

        /**
         * @brief _hessianType Type of Hessian associated to the Task
         */
        HessianType __hessianType;

        /**
         * @brief _A Jacobian of the Task
         */
        Eigen::MatrixXd __A;

        /**
         * @brief _b error associated to the Task
         */
        Eigen::VectorXd __b;

        /**
         * @brief _W Weight multiplied to the task Jacobian
         */
        Eigen::MatrixXd __W;

        /**
         * @brief _lambda error scaling,
         * NOTE:
         *          _lambda >= 0.0
         */
        double __lambda;

        /**
         * @brief _bounds related to the Task
         */
        std::list< ConstraintPtr > __constraints;

        /**
         * @brief _active_joint_mask is vector of bool that represent the active joints of the task.
         * If false the corresponding column of the task jacobian is set to 0.
         */
        std::vector<bool> __active_joints_mask;

        /** Updates the A, b, Aeq, beq, Aineq, b*Bound matrices
            @param x variable state at the current step (input) */
        virtual void _update(const Eigen::VectorXd &x) = 0;

        struct istrue //predicate
        {
           bool operator()(int val) const {return val == true;}
        };

        /**
         * @brief applyActiveJointsMask apply the active joint mask to the A matrix:
         * in tasks in which b does not depend on A, this is done setting to 0 the columns
         * of A corresponding to the index set to false of the _active_joint_mask vector
         * @param A matrix of the Task
         */
        virtual void applyActiveJointsMask(Eigen::MatrixXd& A)
        {
            int rows = A.rows();
            for(unsigned int i = 0; i < __x_size; ++i)
            {
                if(!__active_joints_mask[i])
                    for(unsigned int j = 0; j < rows; ++j)
                        A(j,i) = 0.0;
            }
            //TODO: is necessary here to call update()?
        }

        /**
         * @brief _log can be used to log internal Task variables
         * @param logger a shared pointer to a MatLogger
         */
        virtual void _log(XBot::MatLogger::Ptr logger)
        {

        }

    private:

        /**
         * @brief _WA Jacobian of the Task times the Weight
         */
        mutable Eigen::MatrixXd __WA;

        /**
         * @brief _Wb error associated to the Task times the Weight
         */
        mutable Eigen::VectorXd __Wb;

        /**
         * @brief _Atranspose Jacobian of the task transposed
         */
        mutable Eigen::MatrixXd __Atranspose;
        
        /**
         * @brief ...
         * 
         */
        bool __is_active;
        
        /**
         * @brief ...
         * 
         */
        Eigen::MatrixXd __A_last_active;

    public:
        
    };


 }

#endif
