/**
* inspire_hand_kdl is a ROS package to control Inspire Hand using the KDL library.
* Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
* USA
*/


#ifndef INSPIRE_UNDERACTUATE_H
#define INSPIRE_UNDERACTUATE_H

#include <random>

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <eigen3/Eigen/Core>

#include <inspire_hand_kdl/inspire_kdl_config.h>

using namespace std;
using namespace KDL;

namespace inspire_hand_kdl
{

// Kinematics of inspire_hand_kdl package is a wrapper for
// Orocos KDL's forward and inverse kinematics solvers, both for position and velocity.
// It creates appropriate solvers for each finger using the Inspire hand description.
// Then solvers are used to return the positions and velocities for each joint,
// in order to achieve a desired cartesian-space position and velocity.
// chainfksolverpos_recursive, chainiksolverpos_nr_jl
// and chainiksolvervel_pinv implementations are used.
class Underactuates
{

  private:

  public:

    Underactuates(InspireKdlConfig& kdl_config);
  	~Underactuates();

    void solveUnderactuateJPos(int finger_index, vector<double>& q, bool need_solve=true);
    void solveUnderactuateJVel(int finger_index, const vector<double>& q, Matrix<double, Dynamic, Dynamic>& Jac, bool need_solve=true);
    void solveUnderactuateJPos(int finger_index, const vector<double>& q, vector<double>& dq, Matrix<double, Dynamic, Dynamic>& matM, vector<double>& vecN, bool need_solve=true);
    
};

} // end namespace inspire_hand_kdl

#endif // KINEMATICS_H