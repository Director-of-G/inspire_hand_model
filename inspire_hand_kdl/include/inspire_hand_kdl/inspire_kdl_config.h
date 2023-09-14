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


#ifndef INSPIRE_KDL_CONFIG_H
#define INSPIRE_KDL_CONFIG_H

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>

#include <string>
#include <vector>
#include <numeric>
#include <Eigen/Dense>

#include <kdl_conversions/kdl_msg.h>
#include <geometry_msgs/Quaternion.h>

using namespace std;
using namespace KDL;
using namespace Eigen;

namespace inspire_hand_kdl
{

// InspireKdlConfig of inspire_hand_kdl package is a utility to
// parse KDL tree from INspire hand urdf file and hold it to be
// accessed by solver objects.
class InspireKdlConfig
{

  private:

    vector<Chain*> finger_chain_vec_;
    Tree kdl_tree_;
    std::vector<std::string> FINGER_TIP_LINK{"Link04", "Link64", "Link74", "Link84", "Link48"};

    bool ready_;
    int num_of_fingers_;
    std::vector<int> finger_length_;
    std::vector<int> finger_length_accumulated_;

    void createFingerChains_(const string root_frame);
    void initializeFingerData_();


  public:
  	InspireKdlConfig();
  	~InspireKdlConfig();

    // parse the internal kdl structures using a string, file or ros handle.
    void parseKdl(const ros::NodeHandle& node, const string root_frame="hand_root");
    void parseKdl(const string& str_description, bool from_file, const string root_frame="hand_root");

    // modify the root orientation of kdl chains (useful with mounting)
    void rotateBase(const geometry_msgs::Quaternion& rot);
    void rotateBase(const KDL::Rotation& rot);

    // accessors
    Chain* getFingerChain(const int index) const;
    bool isReady();

    Tree* getTree() {return &kdl_tree_;}
    std::vector<Chain*>& getFingerChains() {return finger_chain_vec_;}
    int getNrOfFingers() {return num_of_fingers_;}
    int getNrOfFingerSegments(int finger_index) {return finger_length_[finger_index];}
    int getNrOfAccumulatedFingerSegments(int finger_index) {return finger_length_accumulated_[finger_index];}
    int getNrOfFreedomFull() {return accumulate(finger_length_.begin(), finger_length_.end(), 0);}

};

} // end namespace inspire_hand_kdl

#endif // INSPIRE_KDL_CONFIG_H