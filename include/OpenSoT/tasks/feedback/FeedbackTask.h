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

#ifndef __FEEDBACK_TASK_H__
#define __FEEDBACK_TASK_H__

#include <OpenSoT/tasks/Aggregated.h>
#include <XBotInterface/ModelInterface.h>
#include <Eigen/Dense>


 namespace OpenSoT {
    namespace tasks {
        namespace feedback {

            /**
             * @brief The FeedbackTask class implements a task that 
             */
            class FeedbackTask : public Task < Eigen::MatrixXd, Eigen::VectorXd > {
            public:
                typedef boost::shared_ptr<FeedbackTask> Ptr;
            protected:
                XBot::ModelInterface& _ref_robot; 
                XBot::ModelInterface& _fb_robot;
                // OpenSoT::tasks::Aggregated::TaskPtr _outTask = NULL;
                // OpenSoT::tasks::Aggregated::Ptr _outAggTask = NULL;

            public:

                /**
                 * @brief FeedbackTask
                 * @param x the initial configuration of the robot
                 * @param robot the robot model
                 */
                FeedbackTask(std::string task_id,
                    const Eigen::VectorXd& x,
                    XBot::ModelInterface& ref_robot,
                    XBot::ModelInterface& fb_robot)
                            : Task(task_id, x.size())
                            , _ref_robot(ref_robot)
                            , _fb_robot(fb_robot)
                {};

                ~FeedbackTask(){};

                virtual void _loop() = 0;

                // virtual const OpenSoT::tasks::Aggregated::TaskPtr checkRefTask(OpenSoT::tasks::Aggregated::TaskPtr refTask){
                //     return refTask->getTaskID() == _outTask->getTaskID() ? this : refTask;

                // };

                virtual const OpenSoT::tasks::Aggregated::TaskPtr checkRefTask(const OpenSoT::tasks::Aggregated::TaskPtr refTask) = 0;

                virtual const OpenSoT::tasks::Aggregated::TaskPtr getModifiedTask() = 0;

                // virtual const OpenSoT::tasks::Aggregated::TaskPtr getModifiedTask(){return _outTask;};

                // virtual const OpenSoT::tasks::Aggregated::Ptr getModifiedAggregatedTask(){return _outAggTask;};

                inline bool isFeedback(){return true;};
                
                // static bool isFeedbackTask(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);

                // static OpenSoT::tasks::feedback::FeedbackTask::Ptr asFeedbackTask(OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr task);
            };
            


        }
    }
 }

#endif
