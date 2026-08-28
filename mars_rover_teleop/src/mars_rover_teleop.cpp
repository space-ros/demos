#define RVIZ_PLUGINLIB_EXPORT __attribute__((visibility("default")))

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <QPushButton>
#include <QGridLayout>

class MarsRoverTeleop : public rviz_common::Panel
{
  Q_OBJECT
public:
  MarsRoverTeleop(QWidget* parent = nullptr)
  : rviz_common::Panel(parent)
  {
    // Initialize ROS node
    node_ = std::make_shared<rclcpp::Node>("mars_rover_teleop_node");

    // Service clients for Wheel Movement
    forward_srv_ = node_->create_client<std_srvs::srv::Empty>("move_forward");
    stop_srv_ = node_->create_client<std_srvs::srv::Empty>("move_stop");
    left_srv_ = node_->create_client<std_srvs::srv::Empty>("turn_left");
    right_srv_ = node_->create_client<std_srvs::srv::Empty>("turn_right");

    // Service clients for Arm Movement
    open_arm_srv_ = node_->create_client<std_srvs::srv::Empty>("open_arm");
    close_arm_srv_ = node_->create_client<std_srvs::srv::Empty>("close_arm");

    // Service clients for Mast Movement
    mast_open_srv_ = node_->create_client<std_srvs::srv::Empty>("mast_open");
    mast_close_srv_ = node_->create_client<std_srvs::srv::Empty>("mast_close");
    mast_rotate_srv_ = node_->create_client<std_srvs::srv::Empty>("mast_rotate");

    // Create buttons for Wheel Movement
    QPushButton* forward_button = new QPushButton("Forward");
    QPushButton* stop_button = new QPushButton("Stop");
    QPushButton* left_button = new QPushButton("Left");
    QPushButton* right_button = new QPushButton("Right");

    // Create buttons for Arm Movement
    QPushButton* open_arm_button = new QPushButton("Open Arm");
    QPushButton* close_arm_button = new QPushButton("Close Arm");

    // Create buttons for Mast Movement
    QPushButton* mast_open_button = new QPushButton("Mast Open");
    QPushButton* mast_close_button = new QPushButton("Mast Close");
    QPushButton* mast_rotate_button = new QPushButton("Mast Rotate");

    // Create a grid layout to arrange buttons
    QGridLayout* layout = new QGridLayout;
    
    // Add Wheel Movement buttons
    layout->addWidget(forward_button, 0, 1);  // Forward button on top
    layout->addWidget(left_button, 1, 0);     // Left button on the left
    layout->addWidget(stop_button, 1, 1);     // Stop button in the center
    layout->addWidget(right_button, 1, 2);    // Right button on the right

    // Add Arm Movement buttons
    layout->addWidget(open_arm_button, 2, 0); // Open Arm button
    layout->addWidget(close_arm_button, 2, 1); // Close Arm button

    // Add Mast Movement buttons
    layout->addWidget(mast_open_button, 3, 0);  // Mast Open button
    layout->addWidget(mast_close_button, 3, 1); // Mast Close button
    layout->addWidget(mast_rotate_button, 3, 2); // Mast Rotate button

    setLayout(layout);

    // Connect buttons to their respective service calls
    connect(forward_button, &QPushButton::clicked, this, &MarsRoverTeleop::callMoveForward);
    connect(stop_button, &QPushButton::clicked, this, &MarsRoverTeleop::callMoveStop);
    connect(left_button, &QPushButton::clicked, this, &MarsRoverTeleop::callTurnLeft);
    connect(right_button, &QPushButton::clicked, this, &MarsRoverTeleop::callTurnRight);
    
    connect(open_arm_button, &QPushButton::clicked, this, &MarsRoverTeleop::callOpenArm);
    connect(close_arm_button, &QPushButton::clicked, this, &MarsRoverTeleop::callCloseArm);
    
    connect(mast_open_button, &QPushButton::clicked, this, &MarsRoverTeleop::callMastOpen);
    connect(mast_close_button, &QPushButton::clicked, this, &MarsRoverTeleop::callMastClose);
    connect(mast_rotate_button, &QPushButton::clicked, this, &MarsRoverTeleop::callMastRotate);
  }

  virtual ~MarsRoverTeleop() override = default;  // virtual destructor
  
private Q_SLOTS:
  void callMoveForward() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    forward_srv_->async_send_request(request);
  }

  void callMoveStop() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    stop_srv_->async_send_request(request);
  }

  void callTurnLeft() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    left_srv_->async_send_request(request);
  }

  void callTurnRight() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    right_srv_->async_send_request(request);
  }

  void callOpenArm() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    open_arm_srv_->async_send_request(request);
  }

  void callCloseArm() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    close_arm_srv_->async_send_request(request);
  }

  void callMastOpen() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    mast_open_srv_->async_send_request(request);
  }

  void callMastClose() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    mast_close_srv_->async_send_request(request);
  }

  void callMastRotate() {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    mast_rotate_srv_->async_send_request(request);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr forward_srv_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_srv_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr left_srv_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr right_srv_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr open_arm_srv_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr close_arm_srv_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr mast_open_srv_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr mast_close_srv_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr mast_rotate_srv_;
};

// Export as plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(MarsRoverTeleop, rviz_common::Panel)

#include "mars_rover_teleop.moc"
