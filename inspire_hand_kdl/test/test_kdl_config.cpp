#include <math.h>

#include <inspire_hand_kdl/inspire_kdl_config.h>

using namespace std;
using namespace inspire_hand_kdl;


void printChainStructure(const KDL::Chain* chain) {
    cout << "Chain structure:" <<std::endl;
    for (Segment seg : chain->segments) {
        cout << "segment " << seg.getName()
             << " joint " << seg.getJoint().getName()
             << std::endl;
    }
}

void printSegmentNum(InspireKdlConfig* config) {
    int num_fingers = config->getNrOfFingers();
    for (int fi=0; fi<num_fingers; fi++) {
        cout << "Finger " << fi
            << " has " << config->getNrOfFingerSegments(fi)
            << " segments and " << config->getNrOfAccumulatedFingerSegments(fi)
            << " segments in total"
            << std::endl;
    }
}

InspireKdlConfig kdl_config_;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kdl_config_test");
    ROS_INFO("KDL config test node");

    ros::NodeHandle nh;
    std::string inspire_urdf_file = "/home/yongpeng/research/projects/dexterous_arm_hand/src/inspire_tactile_hand/urdf/inspire_tactile_hand.urdf";
    kdl_config_.parseKdl(inspire_urdf_file, true, "base_link");

    std::cout << "Number of joints: " << kdl_config_.getTree()->getNrOfJoints() << "." << std::endl;
    std::cout << "Number of segments: " << kdl_config_.getTree()->getNrOfSegments() << "." << std::endl;

    for (int i = 0; i < kdl_config_.getNrOfFingers(); i++) {
        std::cout << "Finger " << i
                    << " has " << kdl_config_.getFingerChains()[i]->getNrOfJoints()
                    << " joints and " << kdl_config_.getFingerChains()[i]->getNrOfJoints()
                    << " segments." <<std::endl;
        printChainStructure(kdl_config_.getFingerChains()[i]);
    }

    std::cout << "Num of freedoms: " << kdl_config_.getNrOfFreedomFull() << std::endl;
    printSegmentNum(&kdl_config_);

    if (kdl_config_.isReady()) {
        ROS_INFO("KDL parse successed!");
    } else {
        ROS_INFO("KDL parse failed!");
    }

    nh.shutdown();
    return 0;
}
