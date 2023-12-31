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

#include "inspire_hand_kdl/kinematics.h"

#include <kdl_conversions/kdl_msg.h>

using namespace inspire_hand_kdl;

/*********************************************************************
*  Kinematics of inspire_hand_kdl package is a wrapper for
*  Orocos KDL's forward and inverse kinematics solvers, both for position and velocity.
*  It creates appropriate solvers for each finger using the Inspire hand description.
*********************************************************************/
Kinematics::Kinematics(InspireKdlConfig& kdl_config,
    const vector<double>& q_min, const vector<double>& q_max,
    int perturbed_trials, unsigned int maxiter, double eps)
    : perturbed_trials_(perturbed_trials), warned_perturbed_trials_(false)
{
  if(!kdl_config.isReady()){
    ROS_ERROR("Kinematics: null kdl config");
    ros::shutdown();
    return;
  }

  kdl_config_ = &kdl_config;
  createFingerSolvers_(kdl_config, q_min, q_max, maxiter, eps);
}
Kinematics::Kinematics(InspireKdlConfig& kdl_config, int perturbed_trials,
    unsigned int maxiter, double eps)
    : perturbed_trials_(perturbed_trials), warned_perturbed_trials_(false)
{
  if(!kdl_config.isReady()){
    ROS_ERROR("Kinematics: null kdl config");
    ros::shutdown();
    return;
  }

  kdl_config_ = &kdl_config;
  // send empty min/max if no joint limit
  vector<double> q_empty;
  createFingerSolvers_(kdl_config, q_empty, q_empty, maxiter, eps);
}
/*********************************************************************
* destructor
*********************************************************************/
Kinematics::~Kinematics()
{
  for (ChainIkSolverPos* obj : finger_ik_pos_){
    delete obj;
  }
  finger_ik_pos_.clear();

  for (ChainIkSolverVel_pinv* obj : finger_ik_vel_){
    delete obj;
  }
  finger_ik_vel_.clear();

  for (ChainFkSolverPos_recursive* obj : finger_fk_pos_){
    delete obj;
  }
  finger_fk_pos_.clear();

  for (ChainJntToJacSolver* obj : finger_solver_vec_){
    delete obj;
  }
  finger_solver_vec_.clear();

  kdl_config_ = nullptr;
}

/*********************************************************************
* It creates appropriate solvers for each finger using the Inspire hand description.
*********************************************************************/
void Kinematics::createFingerSolvers_(InspireKdlConfig& kdl_config,
    const vector<double>& q_min, const vector<double>& q_max,
    unsigned int maxiter, double eps)
{
  // Create kinematics solvers for each finger
  for(int fi=0; fi < kdl_config_->getNrOfFingers(); fi++){

    // Extract the finger chain from whole hand tree
    Chain *finger_chain = kdl_config.getFingerChain(fi);

    // Solvers require the chain and the algorithm parameters
    ChainFkSolverPos_recursive* solver_fk_pos = new ChainFkSolverPos_recursive(*finger_chain);
    ChainIkSolverVel_pinv* solver_ik_vel = new ChainIkSolverVel_pinv(*finger_chain, maxiter, eps);
    ChainJntToJacSolver* solver_vec = new ChainJntToJacSolver(*finger_chain);

    // choose IK with or without limits
    ChainIkSolverPos* solver_ik_pos;
    if(q_min.size() == 0 || q_max.size() == 0){
      // without limits
      solver_ik_pos = new ChainIkSolverPos_LMA(
        *finger_chain,eps, maxiter);

    }else{
      // with limits
      // Create KDL data types
      int finger_length = kdl_config_->getNrOfFingerSegments(fi);
      JntArray qmin_kdl(finger_length);
      JntArray qmax_kdl(finger_length);

      // Copy inputs from arguments
      for(int si=0; si<finger_length; si++){ // si = segment index
        qmin_kdl(si) = q_min[fi * finger_length + si];
        qmax_kdl(si) = q_max[fi * finger_length + si];
      }

      solver_ik_pos = new ChainIkSolverPos_NR_JL(
        *finger_chain, qmin_kdl, qmax_kdl,
        *solver_fk_pos, *solver_ik_vel, maxiter, eps);
    }
    // Fill vectors
    finger_fk_pos_.push_back(solver_fk_pos);
    finger_ik_vel_.push_back(solver_ik_vel);
    finger_ik_pos_.push_back(solver_ik_pos);
    finger_solver_vec_.push_back(solver_vec);

  }
}
/*********************************************************************
* Returns slightly perturbed joint angles; using random perturbation
* limited by the scale param
*********************************************************************/
KDL::JntArray Kinematics::perturbJointAngles_(const KDL::JntArray& q, double scale){
  // create the random distribution
  std::uniform_real_distribution<double> perturb(-scale/2,scale/2);

  // create the output array
  int joint_count = q.rows();
  JntArray q_out(q.rows());

  for(int j=0; j < joint_count; j++)
    q_out(j) = q(j) + perturb(rand_);

  return q_out;
}
/*********************************************************************
* Recursively solve IK until a good solution is obtained or the trial limit reached
*********************************************************************/
int Kinematics::attemptIK_(int finger_index, const JntArray& q_init, const KDL::Frame& x_des, JntArray& q_des, int trial){

  int err = finger_ik_pos_[finger_index]->CartToJnt(q_init, x_des, q_des);
  if(err<0)
    ROS_DEBUG_STREAM("Kinematics: f"<<finger_index<<" "<<finger_ik_pos_[finger_index]->strError(err));

  // if error: re attempt with perturbed q_init (ignoring err <= -100)
  // this can be deactivated with perturbed_trials=0
  if(err < 0 and err > -100 and trial<perturbed_trials_){
    // warn if not done before
    if(!warned_perturbed_trials_){
      ROS_WARN("Kinematics: Re attempting IK with perturbed state. This warning is given once.");
      warned_perturbed_trials_ = true;
    }

    JntArray q_init_pert = perturbJointAngles_(q_init);
    return attemptIK_(finger_index, q_init_pert, x_des, q_des, ++trial);
  }

  return err;
}
/*********************************************************************
* Convert both cart position and velocity (x, xd)
* to joint position and velocity(q, qd).
* q_init is the current joint position.
*********************************************************************/
void Kinematics::toJointSpace(const vector<double>& q_init, const vector<geometry_msgs::Pose>& x_des,
    const vector<geometry_msgs::Twist>& xd_des, vector<double>& q_des, vector<double>& qd_des)
{
  ROS_WARN("Kinematics::toJointSpace is currently implemented based on KDL, thus is undesirable for underactuated systems!");

  // clear output vectors
  q_des.clear();
  qd_des.clear();

  // Repeat for each finger
  for(int fi=0; fi < kdl_config_->getNrOfFingers(); fi++){
    int finger_length = kdl_config_->getNrOfFingerSegments(fi);
    int q_start = kdl_config_->getNrOfAccumulatedFingerSegments(fi);

    // Create KDL data types
      // inputs
    JntArray q_init_kdl(finger_length);
    Frame x_des_kdl;
    Twist xd_des_kdl;
      // output
    JntArray q_des_kdl(finger_length);
    JntArray qd_des_kdl(finger_length);

    // Copy inputs from arguments
    for(int si=0; si<finger_length; si++){ // si = segment index
      q_init_kdl(si) = q_init[q_start + si];
    }
      // twist
    tf::twistMsgToKDL(xd_des[fi], xd_des_kdl);
      // frame
    tf::poseMsgToKDL(x_des[fi], x_des_kdl);

    // Get the inverse kinematics transformations
    finger_ik_vel_[fi]->CartToJnt(q_init_kdl, xd_des_kdl, qd_des_kdl);
    attemptIK_(fi, q_init_kdl, x_des_kdl, q_des_kdl);

    // Add outputs to std vectors
    for(int si=0; si<finger_length; si++){ // si = segment index
      q_des.push_back(q_des_kdl(si));
      qd_des.push_back(qd_des_kdl(si));
    }

  }
}
/*********************************************************************
* Convert cart position (x) to joint position (q).
* q_init is the current joint position.
*********************************************************************/
int Kinematics::calcJointPos(const JntArray& q_init, const vector<KDL::Frame>& x_des, JntArray& q_des)
{
  ROS_WARN("Kinematics::calcJointPos is currently implemented based on KDL, thus is undesirable for underactuated systems!");

  int err_return = 0; // error no, 0 = no error

  int num_dof_full = kdl_config_->getNrOfFreedomFull();
  int num_fingers = kdl_config_->getNrOfFingers();
  q_des.resize(num_dof_full);

  // Repeat for each finger
  for(int fi=0; fi < num_fingers; fi++){
      int finger_length = kdl_config_->getNrOfFingerSegments(fi);
      int q_start = kdl_config_->getNrOfAccumulatedFingerSegments(fi);

      JntArray q_init_finger(finger_length);
      JntArray q_des_finger(finger_length);

      // get finger joints
      for(int si=0; si<finger_length; si++) // si = segment index
        q_init_finger(si) = q_init(q_start + si);

      // Get the inverse kinematics transformation
      int err = attemptIK_(fi, q_init_finger, x_des[fi], q_des_finger);

      // add to return array
      for(int si=0; si<finger_length; si++) { // si = segment index
        q_des(q_start + si) = q_des_finger(si);
      }

      err_return = min(err, err_return); // report the lowest error
  }

  return err_return;
}
int Kinematics::calcJointPos(const vector<double>& q_init, const vector<geometry_msgs::Pose>& x_des, vector<double>& q_des)
{

  int err; // error no, 0 = no error

  // convert to kdl types
  JntArray q_init_kdl;
  kdl_control_tools::vectorStdToKdl(q_init, q_init_kdl);

  vector<KDL::Frame> x_des_kdl(kdl_config_->getNrOfFingers());
  for(int fi=0; fi < kdl_config_->getNrOfFingers(); fi++){
    tf::poseMsgToKDL(x_des[fi], x_des_kdl[fi]);
  }

  // call the core method
  JntArray q_des_kdl;
  err = calcJointPos(q_init_kdl, x_des_kdl, q_des_kdl);

  // convert the output to std
  kdl_control_tools::vectorKdlToStd(q_des_kdl, q_des);

  return err;
}
/*********************************************************************
* Convert cart velocity (xd) to joint velocity (qd).
* q_init is the current joint position.
*********************************************************************/
void Kinematics::calcJointVel(const JntArray& q_init, const vector<KDL::Twist>& xd_des, JntArray& qd_des)
{
  ROS_WARN("Kinematics::calcJointVel is currently implemented based on KDL, thus is undesirable for underactuated systems!");

  int num_dof_full = kdl_config_->getNrOfFreedomFull();
  qd_des.resize(num_dof_full);

  // Repeat for each finger
  for(int fi=0; fi < kdl_config_->getNrOfFingers(); fi++){
      int finger_length = kdl_config_->getNrOfFingerSegments(fi);
      int q_start = kdl_config_->getNrOfAccumulatedFingerSegments(fi);

      JntArray q_init_finger(finger_length);
      JntArray qd_des_finger(finger_length);

      // get finger joints
      for(int si=0; si<finger_length; si++) // si = segment index
        q_init_finger(si) = q_init(q_start + si);

        // Get the inverse kinematics transformations
        finger_ik_vel_[fi]->CartToJnt(q_init_finger, xd_des[fi], qd_des_finger);

      // add to return array
      for(int si=0; si<finger_length; si++) // si = segment index
        qd_des(q_start + si) = qd_des_finger(si);
  }
}
void Kinematics::calcJointVel(const vector<double>& q_init, const vector<geometry_msgs::Twist>& xd_des, vector<double>& qd_des)
{
  // convert to kdl types
  JntArray q_init_kdl;
  kdl_control_tools::vectorStdToKdl(q_init, q_init_kdl);

  vector<KDL::Twist> xd_des_kdl(kdl_config_->getNrOfFingers());
  for(int fi=0; fi < kdl_config_->getNrOfFingers(); fi++){
    tf::twistMsgToKDL(xd_des[fi], xd_des_kdl[fi]);
  }

  // call the core method
  JntArray qd_des_kdl;
  calcJointVel(q_init_kdl, xd_des_kdl, qd_des_kdl);

  // convert the output to std
  kdl_control_tools::vectorKdlToStd(qd_des_kdl, qd_des);

}

/*********************************************************************
* Convert joint pos (q) to cart pos (x) using forward kinematics.
*********************************************************************/
void Kinematics::calcCartPos(const JntArray& q, vector<KDL::Frame>& x){

  // clear output vector
  x.clear();

  // Repeat for each finger
  for(int fi=0; fi < kdl_config_->getNrOfFingers(); fi++){

      // output
    Frame frame;

    // Slice the finger joint angles
    KDL::JntArray q_fi;
    int q_start = kdl_config_->getNrOfAccumulatedFingerSegments(fi);
    int q_end = kdl_config_->getNrOfFingerSegments(fi);
    q_fi.data = q.data.segment(q_start, q_end);

    // Get the forward kinematics transformations
    finger_fk_pos_[fi]->JntToCart(q_fi, frame);

    // Add frames to vector
    x.push_back(frame);
  }
}
void Kinematics::calcCartPos(const vector<double>& q, vector<KDL::Frame>& x){
  // to kdl type
  JntArray q_kdl;
  kdl_control_tools::vectorStdToKdl(q, q_kdl);
  // call the core method
  calcCartPos(q_kdl, x);
}
// wrapper with geometry_msgs
void Kinematics::calcCartPos(const vector<double>& q, vector<geometry_msgs::Pose>& x){
  // clear output vector
  x.clear();

  vector<KDL::Frame> x_kdl;
  calcCartPos(q, x_kdl);

  // convert to std
  for(int fi=0; fi < kdl_config_->getNrOfFingers(); fi++){
    // Add outputs to std vectors
    geometry_msgs::Pose x_finger;
    tf::poseKDLToMsg(x_kdl[fi], x_finger);
    x.push_back(x_finger);
  }
}

/*********************************************************************
* Calculate finger tip jacobian at joint pos (q).
*********************************************************************/
void Kinematics::calcJac(const vector<double>& q){
  finger_tip_jac_.clear();

  JntArray q_kdl;
  kdl_control_tools::vectorStdToKdl(q, q_kdl);

  for (int fi=0; fi<kdl_config_->getNrOfFingers(); fi++) {
    JntArray q_fi;
    int finger_length = kdl_config_->getNrOfFingerSegments(fi);
    int q_start = kdl_config_->getNrOfAccumulatedFingerSegments(fi);

    q_fi.data = q_kdl.data.segment(q_start, finger_length);
    KDL::Jacobian jac(kdl_config_->getNrOfFingerSegments(fi));
    finger_solver_vec_[fi]->JntToJac(q_fi, jac);
    finger_tip_jac_.push_back(jac);
  }
}
