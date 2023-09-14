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

#include "inspire_hand_kdl/inspire_kdl_config.h"

using namespace std;
using namespace KDL;

namespace inspire_hand_kdl
{
/*********************************************************************
* Create an empty Inspire KDL configuration. 'parseKdl' should be used
* for it to function properly.
*********************************************************************/
InspireKdlConfig::InspireKdlConfig()
{
  ready_ = false;
}
/*********************************************************************
* Comment
*********************************************************************/
InspireKdlConfig::~InspireKdlConfig()
{
  for (Chain* obj : finger_chain_vec_){
    delete obj;
  }
  finger_chain_vec_.clear();
}
/*********************************************************************
* Extracts finger chains from a kdl_tree containing all fingers.
*********************************************************************/
void InspireKdlConfig::createFingerChains_(const string root_frame)
{
  auto it = FINGER_TIP_LINK.begin();
  while (it != FINGER_TIP_LINK.end())
  {
    std::string finger_end_link = *it;
    // Extract the finger chain from whole hand tree
    Chain finger_chain;
    kdl_tree_.getChain(root_frame, finger_end_link, finger_chain);

    // Fill vectors
    finger_chain_vec_.push_back(new Chain(finger_chain));

    it++;
  }
  initializeFingerData_();

  ready_ = true;
}

void InspireKdlConfig::initializeFingerData_()
{
  num_of_fingers_ = FINGER_TIP_LINK.size();
  int num_of_finger_segments_accumulated = 0;
  
  for (int i=0; i<num_of_fingers_; i++) {
    int num_of_cur_finger_segments = finger_chain_vec_[i]->getNrOfSegments();

    finger_length_.push_back(num_of_cur_finger_segments);
    finger_length_accumulated_.push_back(num_of_finger_segments_accumulated);

    num_of_finger_segments_accumulated += num_of_cur_finger_segments;
  }
}

/*********************************************************************
* Parses the KDL tree using the robot description in ROS parameter server.
*********************************************************************/
void InspireKdlConfig::parseKdl(const ros::NodeHandle& node, const string root_frame)
{
  string robot_desc_string;
  node.getParam("/robot_description", robot_desc_string);

  parseKdl(robot_desc_string, false, root_frame);
}
/*********************************************************************
* Parses the KDL tree using the given robot description string or filename.
*********************************************************************/
void InspireKdlConfig::parseKdl(const string& str_description, bool from_file, const string root_frame)
{

  // Parse the kdl tree, either from file
  if (from_file){
    ROS_INFO("Parsing kdl from file...");
    if (!kdl_parser::treeFromFile(str_description, kdl_tree_)){
      ROS_ERROR("Failed to construct kdl tree");
      return;
    }
  }
  // or from the description text
  else if (!kdl_parser::treeFromString(str_description, kdl_tree_)){
    ROS_INFO("Parsing kdl from parameter server...");
    ROS_ERROR("Failed to construct kdl tree");
    return;
  }

  createFingerChains_(root_frame);
}
/*********************************************************************
* Modify the root orientation of kdl chains (useful with mounting)
*********************************************************************/
void InspireKdlConfig::rotateBase(const geometry_msgs::Quaternion& rot){
  // convert to kdl and call the base method
  KDL::Rotation rot_kdl;
  tf::quaternionMsgToKDL(rot, rot_kdl);

  rotateBase(rot_kdl);
}
void InspireKdlConfig::rotateBase(const KDL::Rotation& rot){

  // get the previous properties of the base segment
  KDL::Segment *s_prev = &(finger_chain_vec_[0]->segments[0]);

  KDL::Frame f_prev = s_prev->getFrameToTip();
  string name_prev = s_prev->getName();
  KDL::Joint j_prev = s_prev->getJoint();
  KDL::RigidBodyInertia inert_prev = s_prev->getInertia();

  // create a new pose (frame)
  KDL::Frame f_new(rot, f_prev.p);

  // create new segment with the new pose
  KDL::Segment s_new(name_prev, j_prev, f_new, inert_prev);

  // Update the root pose for each finger
  // since they are separate chains
  for(int fi=0; fi < FINGER_TIP_LINK.size(); fi++){

    // replace the previous base segment with the new one
    finger_chain_vec_[fi]->segments[0] = s_new;
  }
}
/*********************************************************************
* Accessor for a finger kdl chain for a given finger index
*********************************************************************/
Chain* InspireKdlConfig::getFingerChain(const int index) const
{
  if(!ready_){
    ROS_WARN("inspire_hand_kdl InspireKdlConfig: Parse a robot description first.");
    return nullptr;
  }

  return finger_chain_vec_[index];

}
/*********************************************************************
* Ready to use?
*********************************************************************/
bool InspireKdlConfig::isReady()
{
  return ready_;
}

} // end namespace inspire_hand_kdl