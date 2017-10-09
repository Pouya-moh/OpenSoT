/*
 * Copyright (C) 2016 Cogimon
 * Author: Enrico Mingo
 * email:  enrico.mingo@iit.it
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

#include <OpenSoT/constraints/torque/TorqueLimits.h>
#include <cmath>

using namespace OpenSoT::constraints::torque;

TorqueLimits::TorqueLimits(const Eigen::VectorXd& torque_max,
                           const Eigen::VectorXd& torque_min) :
    Constraint("torque_limits", torque_max.size()){

    setTorqueLimits(torque_max, torque_min);

}

void TorqueLimits::getTorqueLimits(Eigen::VectorXd& torque_max,
                                   Eigen::VectorXd& torque_min)
{
    torque_max = getUpperBound();
    torque_min = getLowerBound();
}

void TorqueLimits::setTorqueLimits(const Eigen::VectorXd& torque_max,
                                   const Eigen::VectorXd& torque_min)
{
    setUpperBound(torque_max);
    setLowerBound(torque_min);
    
    
}


