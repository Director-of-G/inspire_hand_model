/**
* inspire_hand_kdl is a ROS package to control inspire Hand using the KDL library.
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

#include "inspire_hand_kdl/inspire_underactuate.h"

using namespace inspire_hand_kdl;

Underactuates::Underactuates(InspireKdlConfig& kdl_config)
{

}

Underactuates::~Underactuates()
{

}

void Underactuates::solveUnderactuateJPos(int finger_index, vector<double>& q, bool need_solve=true)
{

}

void Underactuates::solveUnderactuateJVel(int finger_index, const vector<double>& q, Matrix<double, Dynamic, Dynamic>& Jac, bool need_solve=true)
{

}

void Underactuates::solveUnderactuateJPos(int finger_index, const vector<double>& q, vector<double>& dq, Matrix<double, Dynamic, Dynamic>& matM, vector<double>& vecN, bool need_solve=true)
{

}
