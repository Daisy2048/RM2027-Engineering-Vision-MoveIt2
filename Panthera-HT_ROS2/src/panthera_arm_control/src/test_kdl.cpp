#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

int main()
{
    std::string urdf_file = ament_index_cpp::get_package_share_directory("panthera_ht_ros_description")
                            + "/urdf/panthera_ht_ros_description.urdf";
    std::cout << "URDF: " << urdf_file << std::endl;

    KDL::Tree tree;
    if (!kdl_parser::treeFromFile(urdf_file, tree)) {
        std::cerr << "Failed to parse URDF" << std::endl;
        return 1;
    }
    std::cout << "Tree segments: " << tree.getNrOfSegments()
              << ", joints: " << tree.getNrOfJoints() << std::endl;

    // Test both chains
    std::vector<std::string> tips = {"link6", "gripper_center"};
    for (auto & tip : tips) {
        KDL::Chain chain;
        if (!tree.getChain("base_link", tip, chain)) {
            std::cerr << "Failed chain base_link -> " << tip << std::endl;
            continue;
        }
        std::cout << "\n=== Chain: base_link -> " << tip
                  << " (joints=" << chain.getNrOfJoints()
                  << ", segments=" << chain.getNrOfSegments() << ") ===" << std::endl;

        // Print segments
        for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
            auto seg = chain.getSegment(i);
            auto jnt = seg.getJoint();
            auto f = seg.getFrameToTip();
            std::cout << "  Seg[" << i << "]: " << seg.getName()
                      << " joint=" << jnt.getName()
                      << " type=" << jnt.getTypeName()
                      << " tip=(" << f.p.x() << ", " << f.p.y() << ", " << f.p.z() << ")"
                      << std::endl;
        }

        unsigned int nj = chain.getNrOfJoints();
        KDL::ChainFkSolverPos_recursive fk(chain);

        // FK at zero
        KDL::JntArray q_zero(nj);
        KDL::Frame frame;
        fk.JntToCart(q_zero, frame);
        double r, p, y;
        frame.M.GetRPY(r, p, y);
        std::cout << "\nFK at zero: pos=(" << frame.p.x() << ", " << frame.p.y() << ", " << frame.p.z()
                  << ") rpy=(" << r << ", " << p << ", " << y << ")" << std::endl;

        // FK at various configs
        std::vector<std::vector<double>> configs = {
            {0, 0.8, 0.8, 0, 0, 0},
            {0, 1.5, 1.5, 0, 0, 0},
            {0, 1.0, 2.0, 0, 0, 0},
            {0, 0.5, 0.5, 0, 0, 0},
            {0, 1.0, 1.0, 0, 0, 0},
            {0, 2.0, 2.0, 0, 0, 0},
        };
        std::cout << "\nWorkspace exploration:" << std::endl;
        for (auto & cfg : configs) {
            KDL::JntArray q(nj);
            for (unsigned int j = 0; j < nj && j < cfg.size(); ++j)
                q(j) = cfg[j];
            fk.JntToCart(q, frame);
            std::cout << "  q=[";
            for (unsigned int j = 0; j < nj; ++j)
                std::cout << std::fixed << std::setprecision(1) << q(j) << (j < nj-1 ? "," : "");
            std::cout << "] -> pos=(" << std::setprecision(4)
                      << frame.p.x() << ", " << frame.p.y() << ", " << frame.p.z() << ")" << std::endl;
        }

        // IK tests
        Eigen::Matrix<double, 6, 1> L;
        L << 1.0, 1.0, 1.0, 0.01, 0.01, 0.01;
        KDL::ChainIkSolverPos_LMA ik(chain, L, 1e-5, 500);

        std::cout << "\nIK tests:" << std::endl;
        // Test with the zero-position FK result
        KDL::JntArray q_seed(nj), q_out(nj);
        KDL::Frame target;

        // Target 1: near zero pose
        target.p = KDL::Vector(frame.p.x(), frame.p.y(), frame.p.z());
        fk.JntToCart(q_zero, frame);
        target = frame;  // exact zero-position frame
        int ret = ik.CartToJnt(q_seed, target, q_out);
        std::cout << "  IK(zero FK pose): ret=" << ret << std::endl;

        // Target 2: (0.3, 0, 0.2)
        target.p = KDL::Vector(0.3, 0, 0.2);
        target.M = KDL::Rotation::RPY(0, 1.57, 0);
        ret = ik.CartToJnt(q_seed, target, q_out);
        std::cout << "  IK(0.3, 0, 0.2, rpy=0,1.57,0): ret=" << ret;
        if (ret >= 0) {
            std::cout << " q=[";
            for (unsigned int j = 0; j < nj; ++j)
                std::cout << std::setprecision(4) << q_out(j) << (j < nj-1 ? "," : "");
            std::cout << "]";
            // Verify
            fk.JntToCart(q_out, frame);
            std::cout << " verify=(" << frame.p.x() << "," << frame.p.y() << "," << frame.p.z() << ")";
        }
        std::cout << std::endl;

        // Target 3: try something closer
        target.p = KDL::Vector(0.15, 0, 0.15);
        target.M = KDL::Rotation::RPY(0, 0.5, 0);
        ret = ik.CartToJnt(q_seed, target, q_out);
        std::cout << "  IK(0.15, 0, 0.15, rpy=0,0.5,0): ret=" << ret;
        if (ret >= 0) {
            std::cout << " q=[";
            for (unsigned int j = 0; j < nj; ++j)
                std::cout << std::setprecision(4) << q_out(j) << (j < nj-1 ? "," : "");
            std::cout << "]";
            fk.JntToCart(q_out, frame);
            std::cout << " verify=(" << frame.p.x() << "," << frame.p.y() << "," << frame.p.z() << ")";
        }
        std::cout << std::endl;

        // Target 4: try with different seed
        for (unsigned int j = 0; j < nj; ++j) q_seed(j) = 0;
        q_seed(1) = 1.0; q_seed(2) = 1.0;  // better seed
        target.p = KDL::Vector(0.3, 0, 0.2);
        target.M = KDL::Rotation::RPY(0, 1.57, 0);
        ret = ik.CartToJnt(q_seed, target, q_out);
        std::cout << "  IK(0.3, 0, 0.2, seed=[0,1,1,0,0,0]): ret=" << ret;
        if (ret >= 0) {
            std::cout << " q=[";
            for (unsigned int j = 0; j < nj; ++j)
                std::cout << std::setprecision(4) << q_out(j) << (j < nj-1 ? "," : "");
            std::cout << "]";
            fk.JntToCart(q_out, frame);
            std::cout << " verify=(" << frame.p.x() << "," << frame.p.y() << "," << frame.p.z() << ")";
        }
        std::cout << std::endl;
    }

    return 0;
}
