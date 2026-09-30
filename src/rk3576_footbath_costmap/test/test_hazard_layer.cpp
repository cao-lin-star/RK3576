#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include "../src/hazard_layer.cpp"

class HazardApplied : public ::testing::TestWithParam<std::string> {
protected:
  static void SetUpTestSuite() {
    // The executor member is constructed before SetUp().
    if(!rclcpp::ok()) rclcpp::init(0,nullptr);
  }
  std::shared_ptr<nav2_util::LifecycleNode> node;
  rclcpp::Node::SharedPtr source;
  std::unique_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> parent;
  std::unique_ptr<rk3576_footbath_costmap::HazardLayer> layer;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr input;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr output;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<std_msgs::msg::Header> acks;
  int revision=0;

  void spin(unsigned loops=20) {
    for(unsigned i=0;i<loops;++i) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  void SetUp() override {
    if(!rclcpp::ok()) rclcpp::init(0,nullptr);
    node=std::make_shared<nav2_util::LifecycleNode>("hazard_layer_test",GetParam());
    source=std::make_shared<rclcpp::Node>("hazard_layer_source");
    tf=std::make_unique<tf2_ros::Buffer>(node->get_clock());
    parent=std::make_unique<nav2_costmap_2d::LayeredCostmap>("odom",false,false);
    parent->resizeMap(100,100,.05,-2.5,-2.5);
    layer=std::make_unique<rk3576_footbath_costmap::HazardLayer>();
    auto group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    layer->initialize(parent.get(),"hazard",tf.get(),node,group);
    input=source->create_publisher<geometry_msgs::msg::PoseArray>(
      "/safety/hazard_zones",rclcpp::QoS(1).transient_local());
    output=source->create_subscription<std_msgs::msg::Header>(
      GetParam()+"/hazard_zones_applied",rclcpp::QoS(1).reliable(),
      [this](std_msgs::msg::Header::ConstSharedPtr msg){acks.push_back(*msg);});
    executor.add_node(node->get_node_base_interface());
    executor.add_node(source);
    for(unsigned i=0;i<100 && input->get_subscription_count()==0;++i) spin(1);
    ASSERT_EQ(input->get_subscription_count(),1U);
  }
  void TearDown() override {
    executor.remove_node(node->get_node_base_interface());
    executor.remove_node(source);
  }
  void transform() {
    geometry_msgs::msg::TransformStamped message;
    message.header.frame_id="odom"; message.child_frame_id="map";
    message.header.stamp=node->now(); message.transform.rotation.w=1.;
    tf->setTransform(message,"test",true);
  }
  std_msgs::msg::Header send(bool empty=false, bool invalid=false) {
    geometry_msgs::msg::PoseArray message;
    message.header.frame_id=invalid ? "wrong_frame" : "map";
    message.header.stamp.sec=++revision;
    if(!empty) {
      geometry_msgs::msg::Pose zone;
      zone.position.x=.4; zone.position.y=.1; zone.position.z=.15;
      message.poses.push_back(zone);
    }
    input->publish(message); spin();
    return message.header;
  }
  void bounds() {
    double minx=1e9,miny=1e9,maxx=-1e9,maxy=-1e9;
    layer->updateBounds(0,0,0,&minx,&miny,&maxx,&maxy);
  }
  void costs(int max_i=100,int max_j=100) {
    auto grid=parent->getCostmap();
    grid->resetMap(0,0,100,100);
    layer->updateCosts(*grid,0,0,max_i,max_j); spin();
  }
  unsigned lethal() {
    unsigned count=0;
    for(unsigned i=0;i<10000;++i)
      count+=parent->getCostmap()->getCharMap()[i]==nav2_costmap_2d::LETHAL_OBSTACLE;
    return count;
  }
};

TEST_P(HazardApplied, AcknowledgesOnlyAfterCompleteValidCostUpdate) {
  transform();
  auto header=send();
  costs(); EXPECT_TRUE(acks.empty()); // Receipt alone is not application.
  bounds(); EXPECT_TRUE(acks.empty());
  costs(1,1); EXPECT_TRUE(acks.empty()); // A clipped unrelated window is not application.
  costs();
  ASSERT_FALSE(acks.empty()); EXPECT_EQ(acks.back(),header);
  EXPECT_GT(lethal(),0U); EXPECT_TRUE(layer->isCurrent());
}

TEST_P(HazardApplied, DoesNotMixNewInputWithOldPreparedBounds) {
  transform(); send(); bounds();
  const auto newest=send(); costs();
  EXPECT_TRUE(acks.empty());
  bounds(); costs();
  ASSERT_FALSE(acks.empty()); EXPECT_EQ(acks.back(),newest);
}

TEST_P(HazardApplied, TransformFailureCannotBeAcknowledgedAndCanRecover) {
  const auto header=send(); bounds(); costs();
  EXPECT_FALSE(layer->isCurrent()); EXPECT_TRUE(acks.empty());
  transform(); bounds(); costs();
  ASSERT_FALSE(acks.empty()); EXPECT_EQ(acks.back(),header);
}

TEST_P(HazardApplied, ResetRequiresAnotherBoundsPassAndRetainsHazardMemory) {
  transform(); send(); bounds(); costs();
  ASSERT_FALSE(acks.empty()); acks.clear();
  layer->reset(); costs();
  EXPECT_TRUE(acks.empty()); EXPECT_FALSE(layer->isCurrent());
  bounds(); costs(); EXPECT_GT(lethal(),0U); EXPECT_FALSE(acks.empty());
}

TEST_P(HazardApplied, EmptyRevisionClearsMemoryOnlyAfterApplication) {
  transform(); send(); bounds(); costs();
  ASSERT_GT(lethal(),0U); acks.clear();
  const auto cleared=send(true); bounds(); costs();
  ASSERT_FALSE(acks.empty()); EXPECT_EQ(acks.back(),cleared); EXPECT_EQ(lethal(),0U);
}

TEST_P(HazardApplied, InvalidRevisionDoesNotProduceAnAck) {
  transform(); send(false,true); bounds(); costs();
  EXPECT_TRUE(acks.empty()); EXPECT_EQ(lethal(),0U);
}

INSTANTIATE_TEST_SUITE_P(LocalAndGlobalNamespaces,HazardApplied,
  ::testing::Values(std::string("/local_costmap"),std::string("/global_costmap")));

TEST_P(HazardApplied, AcceptsMoreThan128ZonesAndAppliesLastZone) {
  transform();
  geometry_msgs::msg::PoseArray message;
  message.header.frame_id="map";message.header.stamp.sec=++revision;
  geometry_msgs::msg::Pose z;
  z.position.x=.4;z.position.y=.1;z.position.z=.04;
  message.poses.assign(512,z);
  message.poses.back().position.x=-1.;
  message.poses.back().position.y=-1.;
  input->publish(message);spin();bounds();costs();
  ASSERT_FALSE(acks.empty());EXPECT_EQ(acks.back(),message.header);
  unsigned mx,my;
  ASSERT_TRUE(parent->getCostmap()->worldToMap(-1.,-1.,mx,my));
  EXPECT_EQ(parent->getCostmap()->getCost(mx,my),nav2_costmap_2d::LETHAL_OBSTACLE);
}
