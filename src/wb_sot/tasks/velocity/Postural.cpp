/*
 * Copyright (C) 2014 Walkman
 * Author: Alessio Rocchi, Enrico Mingo
 * email:  alessio.rocchi@iit.it, enrico.mingo@iit.it
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

#include <wb_sot/tasks/velocity/Postural.h>
#include <yarp/math/Math.h>
#include <drc_shared/cartesian_utils.h>
#include <exception>
#include <cmath>

using namespace wb_sot::tasks::velocity;
using namespace yarp::math;

Postural::Postural(   const yarp::sig::Vector& x) :
    Task("posture", x, x.size())
{

    _A.resize(_x_size, _x_size);
    _A.eye();

    /* first update. Setting desired pose equal to the actual pose */
    this->setReference(_x0);
    this->update(_x0);

    //_referenceInputPort.open("/wb_sot/tasks/velocity/Postural/" + _task_id + "/set_ref:i");
}

Postural::~Postural()
{
   //_referenceInputPort.close();
}

void Postural::update(const yarp::sig::Vector &x) {

    _x = x;
    /************************* COMPUTING TASK *****************************/

    _A = _A.removeCols(0,6);    // remove floating base
    _A = _A.removeRows(3,3);    // remove orientation

    _b = _x_desired - _x;
    /**********************************************************************/
}

void Postural::setReference(const yarp::sig::Vector& x_desired) {
    _x_desired = x_desired;
}


