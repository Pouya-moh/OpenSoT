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

#ifndef __CONSTRAINT_H__
#define __CONSTRAINT_H__

#include <boost/shared_ptr.hpp>
#include <string>
#include <XBotInterface/Logger.hpp>

 namespace OpenSoT {

 /**
  * @brief The Constraint class describes all the different types of constraints:
  * 1. bounds & bilateral
  * 2. equalities
  * 3. unilateral
  */
    class Constraint {
        
    public:
        
        typedef boost::shared_ptr<Constraint> ConstraintPtr;
        
        Constraint(const std::string constraint_id,
                   const unsigned int x_size) :
            _constraint_id(constraint_id), _x_size(x_size) 
        {}
            
            
        virtual ~Constraint() {}

        const unsigned int getXSize() { return _x_size; }
        virtual const Eigen::VectorXd& getLowerBound() { return _lowerBound; }
        virtual const Eigen::VectorXd& getUpperBound() { return _upperBound; }

        virtual const Eigen::MatrixXd& getAeq() { return _Aeq; }
        virtual const Eigen::VectorXd& getbeq() { return _beq; }

        virtual const Eigen::MatrixXd& getAineq() { return _Aineq; }
        virtual const Eigen::VectorXd& getbLowerBound() { return _bLowerBound; }
        virtual const Eigen::VectorXd& getbUpperBound() { return _bUpperBound; }

        /**
         * @brief isEqualityConstraint
         * @return true if Constraint enforces an equality constraint
         */
        virtual bool isEqualityConstraint() { return _Aeq.rows() > 0; }

        /**
         * @brief isEqualityConstraint
         * @return true if Constraint enforces an inequality constraint
         */
        virtual bool isInequalityConstraint() { return _Aineq.rows() > 0; }

        /**
         * @brief isUnilateralConstraint
         * @return true if the Constraint is an unilateral inequality
         */
        virtual bool isUnilateralConstraint() { return isInequalityConstraint() &&
                                                       (_bLowerBound.size() == 0 || _bUpperBound.size() == 0); }
        /**
         * @brief isBilateralConstraint
         * @return true if the Constraint is a bilateral inequality
         */
        virtual bool isBilateralConstraint() { return isInequalityConstraint() && !isUnilateralConstraint(); }

        /**
         * @brief hasBounds checks whether this Constraint contains a bound (or box constraint),
         *                  meaning the constraint matrix $A^T=\left(I \quad \tilde{A}^T\right)$
         * @return true if Constraint if this constraint contains a bound
         */
        virtual bool hasBounds() { return (_upperBound.size() > 0 || _lowerBound.size() > 0); }

        /**
         * @brief isBound checks whether this Constraint is a bound in the form lowerBound <= x <= upperBound,
         *                meaning the constraint matrix $A = I$ is a box constraint
         * @return true if Constraint is a bound
         */
        virtual bool isBound() { return this->hasBounds() &&
                                        !this->isConstraint(); }

        /**
         * @brief isConstraint checks whether this Constraint is a constraint (i.e., it is not a bound)
         * @return true if the Constraint is not a bound
         */
        virtual bool isConstraint() { return this->isEqualityConstraint() ||
                                             this->isInequalityConstraint(); }

        /**
         * @brief getTaskID return the task id
         * @return a string with the task id
         */
        std::string getConstraintID(){ return _constraint_id; }

        /** Updates the A, b, Aeq, beq, Aineq, b*Bound matrices 
            @param x variable state at the current step (input) */
        virtual void update(const Eigen::VectorXd& x) {}

        /**
         * @brief log logs common Constraint internal variables
         * @param logger a shared pointer to a MathLogger
         */
        virtual void log(XBot::MatLogger::Ptr logger)
        {
            if(_Aeq.rows() > 0 && _Aeq.cols() > 0)
                logger->add(_constraint_id + "_Aeq", _Aeq);
            if(_Aineq.rows() > 0 && _Aineq.cols() > 0)
                logger->add(_constraint_id + "_Aineq", _Aineq);
            if(_beq.size() > 0)
                logger->add(_constraint_id + "_beq", _beq);
            if(_bLowerBound.size() > 0)
                logger->add(_constraint_id + "_bLowerBound", _bLowerBound);
            if(_bUpperBound.size() > 0)
                logger->add(_constraint_id + "_bUpperBound", _bUpperBound);
            if(_upperBound.size() > 0)
                logger->add(_constraint_id + "_upperBound",  _upperBound);
            if(_lowerBound.size() > 0)
                logger->add(_constraint_id + "_lowerBound", _lowerBound);
            _log(logger);
        }
        
    protected:
        
        bool setLowerBound(const Eigen::VectorXd& lb);
        
        bool setUpperBound(const Eigen::VectorXd& ub);
        
        bool setAineq(const Eigen::MatrixXd& Aineq);
        
        bool setLowerBoundIneq(const Eigen::VectorXd& lb);
        
        bool setUpperBoundIneq(const Eigen::VectorXd& ub);
        
        bool setAeq(const Eigen::MatrixXd& Aeq);
        
        bool setbeq(const Eigen::VectorXd& beq);
        
    private:

        /**
         * @brief _constraint_id unique name of the constraint
         */
        std::string _constraint_id;

        /**
         * @brief _x_size size of the controlled variables
         */
        unsigned int _x_size;

        /**
         * @brief _lowerBound lower bounds on controlled variables
         * e.g.:
         *              _lowerBound <= x
         */
        Eigen::VectorXd _lowerBound;

        /**
         * @brief _upperBound upper bounds on controlled variables
         * e.g.:
         *              x <= _upperBound
         */
        Eigen::VectorXd _upperBound;

        /**
         * @brief _Aeq Matrix for equality constraint
         * e.g.:
         *              _Aeq*x = _beq
         */
        Eigen::MatrixXd _Aeq;

        /**
         * @brief _beq constraint vector for equality constraint
         * e.g.:
         *              _Aeq*x = _beq
         */
        Eigen::VectorXd _beq;

        /**
         * @brief _Aineq Matrix for inequality constraint
         * e.g.:
         *              _bLowerBound <= _Aineq*x <= _bUpperBound
         */
        Eigen::MatrixXd _Aineq;

        /**
         * @brief _bLowerBound lower bounds in generic inequality constraints
         * e.g.:
         *              _bLowerBound <= _Aineq*x
         */
        Eigen::VectorXd _bLowerBound;

        /**
         * @brief _bUpperBound upper bounds in generic inequality constraints
         * e.g.:
         *              _Aineq*x <= _bUpperBound
         */
        Eigen::VectorXd _bUpperBound;

        /**
         * @brief _log can be used to log internal Constraint variables
         * @param logger a shared pointer to a MatLogger
         */
        virtual void _log(XBot::MatLogger::Ptr logger)
        {

        }
        
    };
 }

#endif
