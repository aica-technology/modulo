#include <gtest/gtest.h>

#include <fstream>
#include <random>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <trac_ik/trac_ik.hpp>

void test(
    const std::string& name, const std::pair<std::string, std::string>& frames, uint num_samples = 10000,
    double eps = 1e-4, double timeout = 0.005) {
  std::ifstream in(std::string(TEST_FIXTURES) + name + ".urdf");
  std::stringstream buffer;
  buffer << in.rdbuf();
  in.close();
  std::cerr << name << std::endl;

  TRAC_IK::TRAC_IK tracik_solver(frames.first, frames.second, buffer.str(), timeout, eps);
  KDL::Chain chain;
  KDL::JntArray ll, ul;

  ASSERT_TRUE(tracik_solver.getKDLChain(chain));
  ASSERT_TRUE(tracik_solver.getKDLLimits(ll, ul));

  // Set up KDL IK
  KDL::ChainFkSolverPos_recursive fk_solver(chain);// Forward kin. solver
  KDL::ChainIkSolverVel_pinv vik_solver(chain);    // PseudoInverse vel solver
  // Joint Limit Solver
  KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver, 1, eps);
  // 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK)

  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());

  for (uint j = 0; j < nominal.data.size(); j++) {
    nominal(j) = (ll(j) + ul(j)) / 2.0;
  }

  // Create desired number of valid, random joint configurations
  std::vector<KDL::JntArray> JointList;
  KDL::JntArray q(chain.getNrOfJoints());

  std::random_device rd;
  std::mt19937 gen(rd());
  for (uint i = 0; i < num_samples; i++) {
    for (uint j = 0; j < ll.data.size(); j++) {
      std::uniform_real_distribution<double> dist(ll(j), ul(j));
      q(j) = dist(gen);
    }
    JointList.push_back(q);
  }

  std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> start_time;
  std::chrono::duration<double> diff;

  KDL::JntArray result;
  KDL::Frame end_effector_pose;
  int rc;

  double total_time = 0;
  uint success = 0;
  for (uint i = 0; i < num_samples; i++) {
    fk_solver.JntToCart(JointList[i], end_effector_pose);
    result = nominal;// start with nominal
    start_time = std::chrono::system_clock::now();
    do {
      q = result;// when iterating start with last solution
      rc = kdl_solver.CartToJnt(q, end_effector_pose, result);
      diff = std::chrono::system_clock::now() - start_time;
    } while (rc < 0 && diff.count() < timeout);
    total_time += diff.count();
    if (rc >= 0) {
      success++;
    }
  }

  std::cerr << "KDL found " << success << " solutions (" << 100.0 * success / num_samples << "%) with an average of "
            << total_time / num_samples << " secs per sample" << std::endl;

  total_time = 0;
  success = 0;
  for (uint i = 0; i < num_samples; i++) {
    fk_solver.JntToCart(JointList[i], end_effector_pose);
    start_time = std::chrono::system_clock::now();
    rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);
    diff = std::chrono::system_clock::now() - start_time;
    total_time += diff.count();
    if (rc >= 0) {
      success++;
    }
  }

  std::cerr << "TRAC-IK found " << success << " solutions (" << 100.0 * success / num_samples
            << "%) with an average of " << total_time / num_samples << " secs per sample" << std::endl
            << std::endl;
}

TEST(IKTest, IK) {
  test("ur5e", {"world", "ur5e_tool0"});
  // test("xarm", {"world", "link_eef"});
  // test("panda_arm", {"panda_link0", "panda_link8"});
  EXPECT_FALSE(true);
}
