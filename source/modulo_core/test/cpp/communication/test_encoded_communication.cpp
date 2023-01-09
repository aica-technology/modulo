#include <gtest/gtest.h>

#include "test_modulo_core/communication_nodes.hpp"

#define tol 0.001

using namespace modulo_core::communication;
using namespace state_representation;

template<typename PubT, typename RecvT>
void test_cartesian_dist(
    const std::shared_ptr<PubT>& pub_state, const std::shared_ptr<RecvT>& recv_state,
    CartesianStateVariable distance_variable
) {
  EXPECT_EQ(pub_state->get_reference_frame(), recv_state->get_reference_frame());
  if (!recv_state->is_empty()) {
    EXPECT_LT(pub_state->dist(*recv_state, distance_variable), tol);
  }
}

template<typename PubT, typename RecvT>
void test_joint_dist(
    const std::shared_ptr<PubT>& pub_state, const std::shared_ptr<RecvT>& recv_state,
    JointStateVariable distance_variable
) {
  if (!recv_state->is_empty()) {
    EXPECT_EQ(pub_state->get_size(), recv_state->get_size());
    EXPECT_LT(pub_state->dist(*recv_state, distance_variable), tol);
  } else {
    EXPECT_EQ(recv_state->get_size(), 0u);
  }
}

class EncodedCommunicationTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);

    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    clock_ = std::make_shared<rclcpp::Clock>();
  }

  void TearDown() override {
    rclcpp::shutdown();
  }

  void add_nodes(
      const std::string& topic_name, const std::shared_ptr<MessagePairInterface>& pub_message,
      const std::shared_ptr<MessagePairInterface>& sub_message
  ) {
    pub_node_ = std::make_shared<MinimalPublisher<modulo_core::EncodedState>>(topic_name, pub_message);
    sub_node_ = std::make_shared<MinimalSubscriber<modulo_core::EncodedState>>(topic_name, sub_message);
    exec_->add_node(pub_node_);
    exec_->add_node(sub_node_);
  }

  void clear_nodes() {
    pub_node_.reset();
    sub_node_.reset();
  }

  template<typename PubT, typename RecvT>
  void communicate(
      PubT publish_state, RecvT receive_state, bool expect_translation = true,
      std::function<void(const std::shared_ptr<PubT>&, const std::shared_ptr<RecvT>&)> test_function = {}
  ) {
    auto expected_type = receive_state.get_type();
    auto expected_name = expect_translation ? publish_state.get_name() : receive_state.get_name();
    auto expected_empty = expect_translation ? publish_state.is_empty() : receive_state.is_empty();
    auto pub_state = std::make_shared<PubT>(publish_state);
    auto recv_state = std::make_shared<RecvT>(receive_state);
    auto pub_message = make_shared_message_pair(pub_state, this->clock_);
    auto recv_message = make_shared_message_pair(recv_state, this->clock_);
    this->add_nodes("/test_topic", pub_message, recv_message);
    this->exec_->template spin_until_future_complete(
        this->sub_node_->received_future, 500ms
    );
    EXPECT_EQ(recv_state->get_type(), expected_type);
    EXPECT_EQ(recv_state->is_empty(), expected_empty);
    EXPECT_EQ(recv_state->get_name(), expected_name);
    if (test_function) {
      test_function(pub_state, recv_state);
    }
    this->clear_nodes();
  }

  template<typename PubT, typename RecvT>
  void communicate_cartesian_state(CartesianStateVariable distance_variable) {
    // test with empty state
    this->communicate<PubT, RecvT>(PubT("this", "world"), RecvT::Identity("that", "base"), false);
    // test with randomly filled state
    this->communicate<PubT, RecvT>(
        PubT::Random("this", "world"), RecvT::Identity("that", "base"), true,
        [distance_variable](const std::shared_ptr<PubT>& pub_state, const std::shared_ptr<RecvT>& recv_state) {
          test_cartesian_dist(pub_state, recv_state, distance_variable);
        }
    );
  }

  template<typename PubT, typename RecvT>
  void communicate_joint_state(JointStateVariable distance_variable) {
    // test with empty state
    this->communicate<PubT, RecvT>(PubT("this", 2), RecvT::Zero("that", 3), false);
    // test with randomly filled state
    this->communicate<PubT, RecvT>(
        PubT::Random("this", 2), RecvT::Zero("that", 3), true,
        [distance_variable](const std::shared_ptr<PubT>& pub_state, const std::shared_ptr<RecvT>& recv_state) {
          test_joint_dist(pub_state, recv_state, distance_variable);
        }
    );
  }

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::shared_ptr<MinimalPublisher<modulo_core::EncodedState>> pub_node_;
  std::shared_ptr<MinimalSubscriber<modulo_core::EncodedState>> sub_node_;
  std::shared_ptr<rclcpp::Clock> clock_;
};

TEST_F(EncodedCommunicationTest, SameType) {
  this->communicate<State, State>(State(StateType::STATE, "this"), State(StateType::STATE, "that"), false);
  this->communicate<State, State>(State(StateType::STATE, "this", false), State(StateType::STATE, "that"));
  this->communicate<SpatialState, SpatialState>(SpatialState("this"), SpatialState("that"), false);
  this->communicate<SpatialState, SpatialState>(
      SpatialState("this", "world", false), SpatialState("that", "base"), true,
      [](const std::shared_ptr<SpatialState>& pub_state, const std::shared_ptr<SpatialState>& recv_state) {
        EXPECT_EQ(pub_state->get_reference_frame(), recv_state->get_reference_frame());
      }
  );
  this->communicate_cartesian_state<CartesianState, CartesianState>(CartesianStateVariable::ALL);
  this->communicate_cartesian_state<CartesianPose, CartesianPose>(CartesianStateVariable::POSE);
  this->communicate_cartesian_state<CartesianTwist, CartesianTwist>(CartesianStateVariable::TWIST);
  this->communicate_cartesian_state<CartesianAcceleration, CartesianAcceleration>(CartesianStateVariable::ACCELERATION);
  this->communicate_cartesian_state<CartesianWrench, CartesianWrench>(CartesianStateVariable::WRENCH);
  this->communicate_joint_state<JointState, JointState>(JointStateVariable::ALL);
  this->communicate_joint_state<JointPositions, JointPositions>(JointStateVariable::POSITIONS);
  this->communicate_joint_state<JointVelocities, JointVelocities>(JointStateVariable::VELOCITIES);
  this->communicate_joint_state<JointAccelerations, JointAccelerations>(JointStateVariable::ACCELERATIONS);
  this->communicate_joint_state<JointTorques, JointTorques>(JointStateVariable::TORQUES);
  this->communicate<Jacobian, Jacobian>(
      Jacobian::Random("this", 3, "ee", "world"), Jacobian("that", 2, "base", "base"), true,
      [](const std::shared_ptr<Jacobian>& pub_state, const std::shared_ptr<Jacobian>& recv_state) {
        EXPECT_EQ(pub_state->get_frame(), recv_state->get_frame());
        EXPECT_EQ(pub_state->get_reference_frame(), recv_state->get_reference_frame());
        EXPECT_TRUE(pub_state->data().isApprox(recv_state->data()));
      }
  );
}

TEST_F(EncodedCommunicationTest, CompatibleType) {
  // SpatialState is empty constructed and will not be published
  // this->communicate<SpatialState, State>(SpatialState("this", "world"), State(StateType::STATE, "that"));

  // State is compatible with everything
  this->communicate<CartesianState, State>(CartesianState::Random("this"), State(StateType::STATE, "that"));
  this->communicate<CartesianPose, State>(CartesianPose::Random("this"), State(StateType::STATE, "that"));
  this->communicate<JointState, State>(JointState::Random("this", 3), State(StateType::STATE, "that"));
  this->communicate<JointPositions, State>(JointPositions::Random("this", 3), State(StateType::STATE, "that"));
  this->communicate<Jacobian, State>(Jacobian::Random("this", 3, "ee"), State(StateType::STATE, "that"));

  // SpatialState is compatible with Cartesian types
  this->communicate<CartesianState, SpatialState>(CartesianState::Random("this"), SpatialState("that"));
  this->communicate<CartesianPose, SpatialState>(CartesianPose::Random("this"), SpatialState("that"));

  // CartesianState is compatible with all Cartesian types
  this->communicate_cartesian_state<CartesianPose, CartesianState>(CartesianStateVariable::ALL);
  this->communicate_cartesian_state<CartesianTwist, CartesianState>(CartesianStateVariable::ALL);
  this->communicate_cartesian_state<CartesianAcceleration, CartesianState>(CartesianStateVariable::ALL);
  this->communicate_cartesian_state<CartesianWrench, CartesianState>(CartesianStateVariable::ALL);

  // Cartesian types are compatible with CartesianState
  this->communicate_cartesian_state<CartesianState, CartesianPose>(CartesianStateVariable::POSE);
  this->communicate_cartesian_state<CartesianState, CartesianTwist>(CartesianStateVariable::TWIST);
  this->communicate_cartesian_state<CartesianState, CartesianAcceleration>(CartesianStateVariable::ACCELERATION);
  this->communicate_cartesian_state<CartesianState, CartesianWrench>(CartesianStateVariable::WRENCH);

  // JointState is compatible with all joint types
  this->communicate_joint_state<JointPositions, JointState>(JointStateVariable::ALL);
  this->communicate_joint_state<JointVelocities, JointState>(JointStateVariable::ALL);
  this->communicate_joint_state<JointAccelerations, JointState>(JointStateVariable::ALL);
  this->communicate_joint_state<JointTorques, JointState>(JointStateVariable::ALL);

  // joint types are compatible with JointState
  this->communicate_joint_state<JointState, JointPositions>(JointStateVariable::POSITIONS);
  this->communicate_joint_state<JointState, JointVelocities>(JointStateVariable::VELOCITIES);
  this->communicate_joint_state<JointState, JointAccelerations>(JointStateVariable::ACCELERATIONS);
  this->communicate_joint_state<JointState, JointTorques>(JointStateVariable::TORQUES);
}

TEST_F(EncodedCommunicationTest, IncompatibleType) {
  // SpatialState is incompatible with State, JointState and Jacobian
  this->communicate<State, SpatialState>(
      State(StateType::STATE, "this"), SpatialState("that"), false
  );
  this->communicate<JointState, SpatialState>(
      JointState::Random("this", 3), SpatialState("that"), false
  );
  this->communicate<Jacobian, SpatialState>(
      Jacobian("this", 3, "ee"), SpatialState("that"), false
  );

  // Jacobian is incompatible with State, SpatialState, CartesianState and JointState
  this->communicate<State, Jacobian>(
      State(StateType::STATE, "this"), Jacobian("that", 3, "base"), false,
      [](const std::shared_ptr<State>&, const std::shared_ptr<Jacobian>& recv_state) {
        EXPECT_EQ(recv_state->get_frame(), "base");
        EXPECT_NEAR(recv_state->data().norm(), 0, tol);
      }
  );
  this->communicate<SpatialState, Jacobian>(
      SpatialState("this"), Jacobian("that", 3, "base"), false,
      [](const std::shared_ptr<SpatialState>&, const std::shared_ptr<Jacobian>& recv_state) {
        EXPECT_EQ(recv_state->get_frame(), "base");
        EXPECT_NEAR(recv_state->data().norm(), 0, tol);
      }
  );
  this->communicate<CartesianState, Jacobian>(
      CartesianState::Random("this", "world"), Jacobian("that", 3, "base"), false,
      [](const std::shared_ptr<CartesianState>&, const std::shared_ptr<Jacobian>& recv_state) {
        EXPECT_EQ(recv_state->get_frame(), "base");
        EXPECT_NEAR(recv_state->data().norm(), 0, tol);
      }
  );
  this->communicate<JointState, Jacobian>(
      JointState::Random("this", 3), Jacobian("that", 3, "base"), false,
      [](const std::shared_ptr<JointState>&, const std::shared_ptr<Jacobian>& recv_state) {
        EXPECT_EQ(recv_state->get_frame(), "base");
        EXPECT_NEAR(recv_state->data().norm(), 0, tol);
      }
  );
}

TEST_F(EncodedCommunicationTest, IncompatibleTypeCartesian) {
  // CartesianState is incompatible with State, SpatialState, JointState and Jacobian
  this->communicate<State, CartesianState>(
      State(StateType::STATE, "this"), CartesianState::Identity("that", "base"), false,
      [](const std::shared_ptr<State>&, const std::shared_ptr<CartesianState>& recv_state) {
        EXPECT_EQ(recv_state->get_reference_frame(), "base");
        EXPECT_NEAR(recv_state->data().norm(), 1, tol);
      }
  );
  this->communicate<SpatialState, CartesianState>(
      SpatialState("this"), CartesianState::Identity("that", "base"), false,
      [](const std::shared_ptr<SpatialState>&, const std::shared_ptr<CartesianState>& recv_state) {
        EXPECT_EQ(recv_state->get_reference_frame(), "base");
        EXPECT_NEAR(recv_state->data().norm(), 1, tol);
      }
  );
  this->communicate<JointState, CartesianState>(
      JointState::Random("this", 3), CartesianState::Identity("that", "base"), false,
      [](const std::shared_ptr<JointState>&, const std::shared_ptr<CartesianState>& recv_state) {
        EXPECT_EQ(recv_state->get_reference_frame(), "base");
        EXPECT_NEAR(recv_state->data().norm(), 1, tol);
      }
  );
  this->communicate<Jacobian, CartesianState>(
      Jacobian("this", 3, "ee"), CartesianState::Identity("that", "base"), false,
      [](const std::shared_ptr<Jacobian>&, const std::shared_ptr<CartesianState>& recv_state) {
        EXPECT_EQ(recv_state->get_reference_frame(), "base");
        EXPECT_NEAR(recv_state->data().norm(), 1, tol);
      }
  );

  // Cartesian derived types are incompatible between each other
  this->communicate<CartesianTwist, CartesianPose>(CartesianTwist("this"), CartesianPose("that"), false);
  this->communicate<CartesianAcceleration, CartesianPose>(CartesianAcceleration("this"), CartesianPose("that"), false);
  this->communicate<CartesianWrench, CartesianPose>(CartesianWrench("this"), CartesianPose("that"), false);
  this->communicate<CartesianPose, CartesianTwist>(CartesianPose("this"), CartesianTwist("that"), false);
  this->communicate<CartesianAcceleration, CartesianTwist>(
      CartesianAcceleration("this"), CartesianTwist("that"), false
  );
  this->communicate<CartesianWrench, CartesianTwist>(CartesianWrench("this"), CartesianTwist("that"), false);
  this->communicate<CartesianPose, CartesianAcceleration>(CartesianPose("this"), CartesianAcceleration("that"), false);
  this->communicate<CartesianTwist, CartesianAcceleration>(
      CartesianTwist("this"), CartesianAcceleration("that"), false
  );
  this->communicate<CartesianWrench, CartesianAcceleration>(
      CartesianWrench("this"), CartesianAcceleration("that"), false
  );
  this->communicate<CartesianPose, CartesianWrench>(CartesianPose("this"), CartesianWrench("that"), false);
  this->communicate<CartesianTwist, CartesianWrench>(CartesianTwist("this"), CartesianWrench("that"), false);
  this->communicate<CartesianAcceleration, CartesianWrench>(
      CartesianAcceleration("this"), CartesianWrench("that"), false
  );
}

TEST_F(EncodedCommunicationTest, IncompatibleTypeJoint) {
  // JointState is incompatible with State, SpatialState, CartesianState and Jacobian
  this->communicate<State, JointState>(
      State(StateType::STATE, "this"), JointState::Zero("that", 3), false,
      [](const std::shared_ptr<State>&, const std::shared_ptr<JointState>& recv_state) {
        EXPECT_EQ(recv_state->get_size(), 3u);
        EXPECT_NEAR(recv_state->data().norm(), 0, tol);
      }
  );
  this->communicate<SpatialState, JointState>(
      SpatialState("this"), JointState::Zero("that", 3), false,
      [](const std::shared_ptr<SpatialState>&, const std::shared_ptr<JointState>& recv_state) {
        EXPECT_EQ(recv_state->get_size(), 3u);
        EXPECT_NEAR(recv_state->data().norm(), 0, tol);
      }
  );
  this->communicate<CartesianState, JointState>(
      CartesianState::Random("this", "world"), JointState::Zero("that", 3), false,
      [](const std::shared_ptr<CartesianState>&, const std::shared_ptr<JointState>& recv_state) {
        EXPECT_EQ(recv_state->get_size(), 3u);
        EXPECT_NEAR(recv_state->data().norm(), 0, tol);
      }
  );
  this->communicate<Jacobian, JointState>(
      Jacobian("this", 3, "base"), JointState::Zero("that", 3), false,
      [](const std::shared_ptr<Jacobian>&, const std::shared_ptr<JointState>& recv_state) {
        EXPECT_EQ(recv_state->get_size(), 3u);
        EXPECT_NEAR(recv_state->data().norm(), 0, tol);
      }
  );

  // Joint derived types are incompatible between each other
  this->communicate<JointVelocities, JointPositions>(JointVelocities("this", 2), JointPositions("that", 3), false);
  this->communicate<JointAccelerations, JointPositions>(
      JointAccelerations("this", 2), JointPositions("that", 3), false
  );
  this->communicate<JointTorques, JointPositions>(JointTorques("this", 2), JointPositions("that", 3), false);
  this->communicate<JointPositions, JointVelocities>(JointPositions("this", 2), JointVelocities("that", 3), false);
  this->communicate<JointAccelerations, JointVelocities>(
      JointAccelerations("this", 2), JointVelocities("that", 3), false
  );
  this->communicate<JointTorques, JointVelocities>(JointTorques("this", 2), JointVelocities("that", 3), false);
  this->communicate<JointPositions, JointAccelerations>(
      JointPositions("this", 2), JointAccelerations("that", 3), false
  );
  this->communicate<JointVelocities, JointAccelerations>(
      JointVelocities("this", 2), JointAccelerations("that", 3), false
  );
  this->communicate<JointTorques, JointAccelerations>(
      JointTorques("this", 2), JointAccelerations("that", 3), false
  );
  this->communicate<JointPositions, JointTorques>(JointPositions("this", 2), JointTorques("that", 3), false);
  this->communicate<JointVelocities, JointTorques>(JointVelocities("this", 2), JointTorques("that", 3), false);
  this->communicate<JointAccelerations, JointTorques>(
      JointAccelerations("this", 2), JointTorques("that", 3), false
  );
}
