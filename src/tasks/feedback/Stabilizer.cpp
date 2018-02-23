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

#include <OpenSoT/tasks/feedback/Stabilizer.h>
#include <OpenSoT/utils/cartesian_utils.h>
#include <kdl/frames.hpp>

#include <exception>
#include <cmath>

using namespace OpenSoT::tasks::feedback;



Stabilizer::Stabilizer( const Eigen::VectorXd& x,
                        XBot::ModelInterface& ref_robot,
                        XBot::ModelInterface& fb_robot,
                        OpenSoT::tasks::velocity::Cartesian::Ptr waist,
                        OpenSoT::tasks::velocity::Cartesian::Ptr l_sole,
                        OpenSoT::tasks::velocity::Cartesian::Ptr r_sole,
                        OpenSoT::tasks::velocity::CoM::Ptr com)
    : FeedbackTask("Stabilizer", x, ref_robot, fb_robot)
    , _waist_task(waist)
    , _l_sole_task(l_sole)
    , _r_sole_task(r_sole)
    , _com_task(com)
    , _Kd(3500.0, 1500.0, 0.0)
    , _Bd(1500.0, 1200.0, 0.0)
    , _enableX(false)
    , _enableY(false)
{
    _des_CoM_Pos = _des_l_sole_Pos = _des_l_sole_Pos = _delta_com = Eigen::Vector3d::Zero();
    _des_Waist_Rot = Eigen::Matrix3d::Identity();

    _des_Waist_Pose = Eigen::Affine3d::Identity();

    tx_old = ty_old = xd_old = yd_old = zd_old = 0.0;

    if (_com_task) {
        // _outTask.reset(new OpenSoT::tasks::velocity::CoM(x, ref_robot));
        _outTask->setLambda(_com_task->getLambda());
    }
    else {
        _outTask.reset(new OpenSoT::tasks::velocity::Cartesian("Cartesian::waist", x, ref_robot, "Waist", "world"));
        _outTask->setLambda(_waist_task->getLambda());
    }

    _lambda = 0.0;
    int rows = _outTask->getA().rows();
    int cols = _outTask->getA().cols();
    _A.resize(rows, cols);
    // _A.setZero(rows, cols);
    _A = _outTask->getA();
    _b.setZero(rows);
    _W.resize(rows, rows);
    _W.setIdentity(rows, rows);
    _W *= 1e-10;
    _hessianType = HST_SEMIDEF;
}

Stabilizer::~Stabilizer()
{
}

const OpenSoT::tasks::Aggregated::TaskPtr Stabilizer::checkRefTask(const OpenSoT::tasks::Aggregated::TaskPtr refTask)
{
    if (refTask->getTaskID() != _outTask->getTaskID()) {
        XBot::Logger::error("Wrong refTask for Stabilizer \n");
        XBot::Logger::error("Need TaskID \'%s\', but TaskID \'%s\'' passed.\n", _outTask->getTaskID().c_str(), refTask->getTaskID().c_str());
    }
    else return shared_from_this();
}

void Stabilizer::_update(const Eigen::VectorXd &x)
{

    /************************* COMPUTING TASK *****************************/
    _des_Waist_Pose.matrix() = _waist_task->getReference();
    _des_l_sole_Pose.matrix() = _l_sole_task->getReference();
    _des_r_sole_Pose.matrix() = _r_sole_task->getReference();

    if (_com_task) {
        _des_CoM_Pos = _com_task->getReference();
    }
    else {
        _des_CoM_Pos = _des_Waist_Pose.translation();
    }

    _des_Waist_Rot = _des_Waist_Pose.linear();

    _loop();

    if (_com_task) {
        // _outTask->setReference(_des_CoM_Pos + _delta_com);
    }
    else {
        _des_Waist_Pose.translation() += _delta_com;
        _outTask->setReference(_des_Waist_Pose.matrix());
    }


    // if (_enableX || _enableY)
    // {
    //     std::cout << "enableX " << _enableX << " enableY " << _enableY << std::endl;
    //     std::cout << "_des_CoM_Pos.transpose() " << _des_CoM_Pos.transpose() << std::endl;
    //     std::cout << "_delta_com.transpose() " << _delta_com.transpose() << std::endl;
    // }
    /**********************************************************************/
}

void Stabilizer::_loop()
{
    double torque_x = 0; // torque_x
    double torque_y = 0; // torque_y
    double xd = 0;
    double yd = 0;
    double x0 = 0;
    double y0 = 0;
    double dT = 0.001;
    double m = _fb_robot.getMass();
    double g = 9.806;
    double mZc = _des_CoM_Pos[2];

    Eigen::Vector6d l_leg_ft, r_leg_ft;
    Eigen::Matrix3d Rpelvis_abs;
    Eigen::Vector3d LnAcc, AgVel;
    _fb_robot.getForceTorque()["l_leg_ft"]->getWrench(l_leg_ft);
    _fb_robot.getForceTorque()["r_leg_ft"]->getWrench(r_leg_ft);
    _fb_robot.getImu()["imu_link"]->getImuData(Rpelvis_abs, LnAcc, AgVel);

    Eigen::Vector3d EulerAng(0, 0, 0);
    EulerAng[0] = std::atan2(Rpelvis_abs(2, 1), Rpelvis_abs(2, 2));
    EulerAng[1] = std::asin(-Rpelvis_abs(2, 0));
    EulerAng[2] = std::atan2(Rpelvis_abs(1, 0), Rpelvis_abs(0, 0));

    // Eigen::Matrix3d IMU_rel = Ry(EulerAng[1]) * Rx(EulerAng[0]);
    // Eigen::Matrix3d Rot_lft2hip = ;
    // Eigen::Matrix3d Rot_rft2hip = ;
    // Eigen::Matrix3d Rot_lft2world = IMU_rel * Rot_lft2hip;
    // Eigen::Matrix3d Rot_rft2world = IMU_rel * Rot_lft2hip;

    // Eigen::Vector3d tmp_Vector3d;
    // Eigen::Matrix3d tmp_Matrix3d;
    Eigen::Affine3d tmp_Affine3d;
    KDL::Frame _tmp_Frame;

    _fb_robot.getPose("l_leg_ft", _tmp_Frame);
    tf::transformKDLToEigen(_tmp_Frame, tmp_Affine3d);
    Eigen::Matrix3d Rot_lft2world = Rz(EulerAng[2]).transpose() * tmp_Affine3d.linear();
    Eigen::Vector3d l_sole_local = Rz(EulerAng[2]).transpose() * tmp_Affine3d.translation();

    _fb_robot.getPose("r_leg_ft", _tmp_Frame);
    tf::transformKDLToEigen(_tmp_Frame, tmp_Affine3d);
    Eigen::Matrix3d Rot_rft2world = Rz(EulerAng[2]).transpose() * tmp_Affine3d.linear();
    Eigen::Vector3d r_sole_local = Rz(EulerAng[2]).transpose() * tmp_Affine3d.translation();

    Eigen::Vector3d gGeomCenLft = 0.5 * (l_sole_local - r_sole_local);
    Eigen::Vector3d gGeomCenRft = 0.5 * (r_sole_local - l_sole_local);

    gGeomCenLft[2] = 0;
    gGeomCenRft[2] = 0;

    Eigen::Vector3d lft_Force = Rot_lft2world * l_leg_ft.head(3);
    Eigen::Vector3d rft_Force = Rot_rft2world * r_leg_ft.head(3);
    Eigen::Vector3d txy_act = Rot_lft2world * l_leg_ft.tail(3)
                              + Rot_rft2world * r_leg_ft.tail(3)
                              + (gGeomCenLft).cross(lft_Force)
                              + (gGeomCenRft).cross(rft_Force) ;

    double TurnYaw = std::atan2(_des_Waist_Rot(1, 0), _des_Waist_Rot(0, 0));
    Eigen::Matrix3d TurnYawO = Rz(TurnYaw);
    Eigen::Vector3d LftRef = TurnYawO.transpose() * _des_l_sole_Pose.translation();
    Eigen::Vector3d RftRef = TurnYawO.transpose() * _des_r_sole_Pose.translation();
    Eigen::Vector3d ComRef = TurnYawO.transpose() * _des_CoM_Pos;

    // only for flat terrain
    double Lsup = 1.0, Rsup = 1.0;
    if (LftRef[2] != 0.0) Lsup = 0.0;
    if (RftRef[2] != 0.0) Rsup = 0.0;
    if (Lsup == 0.0 && Rsup == 0.0) Lsup = Rsup = 1.0;
    Eigen::Vector3d ZmpRef = (Lsup * LftRef + Rsup * RftRef) / (Lsup + Rsup);

    Eigen::Vector3d GeomCenRef = 0.5 * (LftRef + RftRef);
    GeomCenRef[2] = 0.0;

    Eigen::Vector3d ComLocalRef = ComRef - GeomCenRef;
    Eigen::Vector3d cZmpRef = - ComRef + ZmpRef;

    double td_x = m * g * (cZmpRef[1] + ComLocalRef[1] + yd_old);
    double td_y = m * g * (cZmpRef[0] + ComLocalRef[0] + xd_old);

    torque_x = txy_act[0] - td_x;
    torque_y = txy_act[1] + td_y;

    Eigen::Vector3d A;
    // sagittal
    A[0] = - torque_y / mZc;
    xd = (dT / (_Kd[0] * dT + _Bd[0])) * A[0] + _Bd[0] / (_Kd[0] * dT + _Bd[0]) * xd_old;

    // lateral
    A[1] = torque_x / mZc;
    yd = (dT / (_Kd[1] * dT + _Bd[1])) * A[1] + _Bd[1] / (_Kd[1] * dT + _Bd[1]) * yd_old;


    double Tf = 1.0 / 3.0;
    if (_enableX) {
        _delta_com[0] = xd;
    }
    else {
        _delta_com[0] = (Tf * _delta_com[0]) / (Tf + dT);
    }

    if (_enableY) {
        _delta_com[1] = yd;
    }
    else {
        _delta_com[1] = (Tf * _delta_com[1]) / (Tf + dT);
    }

    clamp(_delta_com[0], -0.1, 0.1);
    clamp(_delta_com[1], -0.15, 0.15);

    xd_old = _delta_com[0];
    yd_old = _delta_com[1];

    tx_old = torque_x;
    ty_old = torque_y;

    _delta_com = TurnYawO * _delta_com;
}


template <typename T>
inline void Stabilizer::clamp(T &num, const T& min, const T& max)
{
    num = (num > max) ? max : num;
    num = (num < min) ? min : num;
};


Eigen::Matrix3d Stabilizer::Rz(double angle)
{
    double ct = std::cos(angle);
    double st = std::sin(angle);

    Eigen::Matrix3d result;

    result << ct,  -st,  0,
           st,   ct,  0,
           0,   0,   1;

    return result;
}

