/*
 * Copyright (C) 2017 IIT-ADVR
 * Authors: Chengxu Zhou
 * email:  chengxu.zhou@iit.it
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

#ifndef __TASKS_FEEDBACK_STABILIZER_H__
#define __TASKS_FEEDBACK_STABILIZER_H__


#include <OpenSoT/tasks/feedback/FeedbackTask.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>

#include <boost/enable_shared_from_this.hpp>

 namespace OpenSoT {
    namespace tasks {
        namespace feedback {
            /**
             * @brief The Stabilizer class implements a task that tries to impose a position
             * modification of the CoM w.r.t. the world frame.
             */
            class Stabilizer : public FeedbackTask 
            , public boost::enable_shared_from_this<Stabilizer>{
            public:
                typedef boost::shared_ptr<Stabilizer> Ptr;
            private:
                // OpenSoT::tasks::velocity::CoM::Ptr _outTask = NULL;
                OpenSoT::tasks::velocity::Cartesian::Ptr _outTask = NULL;
                // OpenSoT::tasks::Aggregated::TaskPtr _outTask = NULL;

                OpenSoT::tasks::velocity::Cartesian::Ptr _l_sole_task = NULL;
                OpenSoT::tasks::velocity::Cartesian::Ptr _r_sole_task = NULL;
                OpenSoT::tasks::velocity::Cartesian::Ptr _waist_task = NULL;
                OpenSoT::tasks::velocity::CoM::Ptr _com_task = NULL;

                Eigen::Vector3d _des_CoM_Pos;
                Eigen::Vector3d _des_l_sole_Pos;
                Eigen::Vector3d _des_r_sole_Pos;
                Eigen::Vector3d _delta_com;

                Eigen::Matrix3d _des_Waist_Rot;

                Eigen::Affine3d _des_Waist_Pose;
                Eigen::Affine3d _des_l_sole_Pose;
                Eigen::Affine3d _des_r_sole_Pose;

                double tx_old, ty_old, xd_old, yd_old, zd_old;
                bool _enableX, _enableY;
                Eigen::Vector3d _Kd, _Bd;

                void _loop();

                template <typename T>
                inline void clamp(T &num, const T& min, const T& max);

                Eigen::Matrix3d Rz(double angle);

            public:

                /**
                 * @brief Stabilizer
                 * @param x the initial configuration of the robot
                 * @param robot the robot model
                 */
                Stabilizer(const Eigen::VectorXd& x,
                    XBot::ModelInterface& ref_robot,
                    XBot::ModelInterface& fb_robot, 
                    OpenSoT::tasks::velocity::Cartesian::Ptr waist, 
                    OpenSoT::tasks::velocity::Cartesian::Ptr l_sole, 
                    OpenSoT::tasks::velocity::Cartesian::Ptr r_sole,
                    OpenSoT::tasks::velocity::CoM::Ptr com = NULL);

                ~Stabilizer();

                const OpenSoT::tasks::Aggregated::TaskPtr checkRefTask(const OpenSoT::tasks::Aggregated::TaskPtr refTask);

                const OpenSoT::tasks::Aggregated::TaskPtr getModifiedTask(){return _outTask;};

                void Enable(bool enableX, bool enableY){_enableX = enableX; _enableY = enableY;};

                void _update(const Eigen::VectorXd& x);
            };

        }
    }
 }

#endif
