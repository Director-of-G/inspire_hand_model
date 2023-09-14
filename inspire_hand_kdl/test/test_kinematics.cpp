#include <math.h>
#include <iostream>

#include <inspire_hand_kdl/kinematics.h>

using namespace std;
using namespace inspire_hand_kdl;


InspireKdlConfig kdl_config_;

vector<double> q_test_{0.12, 0.005, 0.03, 0.13,
                       0.06, 0.002, 0.01, 0.09,
                       0.35, 0.004, 0.12, 0.31,
                       0.28, 0.001, 0.37, 0.22,
                       0.15, 0.002, 0.29, 0.13, 0.06, 0.002, 0.49, 0.19};

IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

void printKDLFrame(vector<KDL::Frame>& frame) {
    for (KDL::Frame frm:frame) {
        cout << frm.p.data[0] << " " << frm.p.data[1] << " " << frm.p.data[2] << endl;
        for (int i=0; i<9; i++) {
            cout << frm.M.data[i] << " ";
        }
        cout << endl;
    }
}

void printJacobian(vector<KDL::Jacobian>& Jacobian) {
    for (KDL::Jacobian J:Jacobian) {
        cout << J.data.format(HeavyFmt) << endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinematics_test");
    ROS_INFO("Kinematics test node");

    ros::NodeHandle nh;
    std::string inspire_urdf_file = "/home/yongpeng/research/projects/dexterous_arm_hand/src/inspire_tactile_hand/urdf/inspire_tactile_hand.urdf";
    kdl_config_.parseKdl(inspire_urdf_file, true, "base_link");

    if (kdl_config_.isReady()) {
        ROS_INFO("KDL parse successed!");
    } else {
        ROS_INFO("KDL parse failed!");
    }

    Kinematics kin_inspire_(kdl_config_);
    ROS_INFO("Kinematics initialization successed!");

    vector<KDL::Frame> x_test;
    kin_inspire_.calcCartPos(q_test_, x_test);
    printKDLFrame(x_test);

    kin_inspire_.calcJac(q_test_);
    vector<KDL::Jacobian> Jacobian_rst;
    kin_inspire_.getJac(Jacobian_rst);
    printJacobian(Jacobian_rst);

    nh.shutdown();
    return 0;
}
