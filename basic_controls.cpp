//marker and box move together in a grid :: 
#include "interactive_marker_tutorials/basic_controls.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include <sstream>

namespace interactive_marker_tutorials
{
  // Constructor for BasicControlsPanel
  BasicControlsPanel::BasicControlsPanel(QWidget *parent)
      : rviz_common::Panel(parent), button_(nullptr), basic_controls_node_(nullptr)
  {
    initializePanel();
  }

  void BasicControlsPanel::initializePanel()
  {
    // Initialize the button and connect the signal
    button_ = new QPushButton("Create Marker", this);
    connect(button_, &QPushButton::clicked, this, &BasicControlsPanel::onButtonClicked);

    // Initialize the input fields for frame names
    frame_name_input_ = new QLineEdit(this);
    frame_name_input_->setPlaceholderText("Enter Frame Name");

    parent_frame_name_input_ = new QLineEdit(this);
    parent_frame_name_input_->setPlaceholderText("Enter Parent Frame Name");

    // Initialize the "Publish Frame" button
    publish_frame_button_ = new QPushButton("Publish Frame", this);
    connect(publish_frame_button_, &QPushButton::clicked, this, &BasicControlsPanel::onPublishFrameClicked);

    // Create and link the node for controlling markers
    basic_controls_node_ = std::make_shared<BasicControlsNode>();

    if (basic_controls_node_)
    {
      RCLCPP_INFO(rclcpp::get_logger("BasicControlsPanel"), "Node initialized successfully.");
    }

    // Set layout for the panel
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(button_);
    layout->addWidget(frame_name_input_);
    layout->addWidget(parent_frame_name_input_);
    layout->addWidget(publish_frame_button_);
    setLayout(layout);
    setFixedSize(400, 400);
  }

  void BasicControlsPanel::onButtonClicked()
  {
    // Ensure that the node is set and call the marker creation method
    if (basic_controls_node_)
    {
      RCLCPP_INFO(rclcpp::get_logger("BasicControlsPanel"), "Button clicked: Creating 5x5 Grid of Markers");
      basic_controls_node_->createGridOfBoxes();
      // basic_controls_node_->make6DofMarker(true, 0, tf2::Vector3(0.0, 0.0, 0.0), true); // b1
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("BasicControlsPanel"), "BasicControlsNode is not set.");
    }
  }

  void BasicControlsPanel::onPublishFrameClicked()
  {
    // Ensure that the node is set and get the frame names from input fields
    if (basic_controls_node_)
    {
      std::string frame_name = frame_name_input_->text().toStdString();
      std::string parent_frame_name = parent_frame_name_input_->text().toStdString();

      if (!frame_name.empty() && !parent_frame_name.empty())
      {
        RCLCPP_INFO(rclcpp::get_logger("BasicControlsPanel"), "Publishing transformation between '%s' and '%s'.", frame_name.c_str(), parent_frame_name.c_str());
        basic_controls_node_->publishFrameTransformation(frame_name, parent_frame_name);
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("BasicControlsPanel"), "Frame names are empty.");
      }
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("BasicControlsPanel"), "BasicControlsNode is not set.");
    }
  }

  void BasicControlsPanel::setBasicControlsNode(BasicControlsNode *node)
  {
    basic_controls_node_ = std::shared_ptr<BasicControlsNode>(node);
    if (basic_controls_node_)
    {
      RCLCPP_INFO(rclcpp::get_logger("BasicControlsPanel"), "BasicControlsNode successfully linked to BasicControlsPanel.");
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("BasicControlsPanel"), "Failed to link BasicControlsNode to BasicControlsPanel.");
    }
  }

  BasicControlsPanel::~BasicControlsPanel()
  {
    // Optional: Add any custom cleanup code if needed
  }

  BasicControlsNode::BasicControlsNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("basic_controls", options), menu_handler_()
  {
    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
        "basic_controls",
        get_node_base_interface(),
        get_node_clock_interface(),
        get_node_logging_interface(),
        get_node_topics_interface(),
        get_node_services_interface());

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    frame_timer_ = create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&BasicControlsNode::frameCallback, this));
  }

  visualization_msgs::msg::Marker BasicControlsNode::makeBox(const visualization_msgs::msg::InteractiveMarker &msg)
  {
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;
    return marker;
  }

  void BasicControlsNode::publishFrameTransformation(const std::string &frame_id, const std::string &parent_frame_id)
  {
    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = rclcpp::Clock().now();
    transformStamped.header.frame_id = parent_frame_id;
    transformStamped.child_frame_id = frame_id;

    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0); // No rotation for simplicity
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transformStamped);
  }

  void BasicControlsNode::createGridOfBoxes()
  {
    if (!server_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("BasicControlsNode"), "Interactive Marker server is not initialized.");
      return;
    }

    const int rows = 5;
    const int cols = 5;
    const double spacing = 2.0;

    for (int i = 0; i < rows; ++i)
    {
      for (int j = 0; j < cols; ++j)
      {
        tf2::Vector3 position(i * spacing, j * spacing, 0);
        make6DofMarker(true, 0, position, true); // b1
      }
    }
    server_->applyChanges();
    RCLCPP_INFO(rclcpp::get_logger("BasicControlsNode"), "Markers applied to server.");
  }

  /// dfk//
  void BasicControlsNode::processBoxClick(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
  {
    if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK)
    {
      RCLCPP_INFO(rclcpp::get_logger("basic_controls"), "Marker clicked: %s", feedback->marker_name.c_str());

      // Retrieve the clicked marker
      visualization_msgs::msg::InteractiveMarker int_marker;
      if (!server_->get(feedback->marker_name, int_marker))
      {
        RCLCPP_WARN(get_logger(), "Marker '%s' not found!", feedback->marker_name.c_str());
        return;
      }
      server_->erase(feedback->marker_name); // This will erase the previous marker

      // Extract position from feedback->pose and convert to tf2::Vector3
      tf2::Vector3 position(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
      make6DofMarker(true, visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D, position, true);

      // Update the marker on the server
      // server_->insert(int_marker);
      server_->applyChanges();

      RCLCPP_INFO(get_logger(), "6-DOF marker created at position: [%f, %f, %f]",
                  position.x(), position.y(), position.z());
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("basic_controls"), "Unexpected event type: %d", feedback->event_type);
    }
  }

  void BasicControlsNode::make6DofMarker(bool fixed, unsigned int interaction_mode, const tf2::Vector3 &position, bool show_6dof)
  {
    // Suppress warnings for unused parameters
    (void)fixed;
    (void)show_6dof;

    // Define an interactive marker
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link"; // Adjust frame as needed
    int_marker.header.stamp = this->get_clock()->now();
    int_marker.name = "6dof_marker_" + std::to_string(position.x()) + "_" + std::to_string(position.y());
    int_marker.description = "6-DOF Control Marker";
    int_marker.scale = 1.0; // Adjust marker scale
    int_marker.pose.position.x = position.x();
    int_marker.pose.position.y = position.y();
    int_marker.pose.position.z = position.z();

    // Define controls for moving and rotating along axes
    visualization_msgs::msg::InteractiveMarkerControl control;

    if (interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D)
    {
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
      control.name = "move_3d";
      int_marker.controls.push_back(control);
    }
    else if (interaction_mode == visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D)
    {
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_3D;
      control.name = "rotate_3d";
      int_marker.controls.push_back(control);
    }
    else
    {
      // Add 6-DOF controls (translation + rotation on all axes)
      std::vector<std::string> axes = {"x", "y", "z"};
      for (const std::string &axis : axes)
      {
        control.orientation.w = 1.0;
        if (axis == "x")
        {
          control.orientation.x = 1.0;
          control.orientation.y = 0.0;
          control.orientation.z = 0.0;
          control.name = "rotate_x";
          control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
          int_marker.controls.push_back(control);

          control.name = "move_x";
          control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
          int_marker.controls.push_back(control);
        }
        else if (axis == "y")
        {
          control.orientation.x = 0.0;
          control.orientation.y = 1.0;
          control.orientation.z = 0.0;
          control.name = "rotate_y";
          control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
          int_marker.controls.push_back(control);

          control.name = "move_y";
          control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
          int_marker.controls.push_back(control);
        }
        else if (axis == "z")
        {
          control.orientation.x = 0.0;
          control.orientation.y = 0.0;
          control.orientation.z = 1.0;
          control.name = "rotate_z";
          control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
          int_marker.controls.push_back(control);

          control.name = "move_z";
          control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
          int_marker.controls.push_back(control);
        }
      }
    }
    // Add the cube to the marker
    visualization_msgs::msg::Marker cube_marker = makeBox(int_marker); // Call the makeBox function to create a cube
    visualization_msgs::msg::InteractiveMarkerControl cube_control;
    cube_control.name = "cube_control";
    cube_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
    cube_control.markers.push_back(cube_marker); // Add the cube marker to the control
    int_marker.controls.push_back(cube_control);

    // Insert the marker into the server
    server_->insert(int_marker);

    // Apply changes to ensure the marker is updated in RViz
    server_->applyChanges();

    RCLCPP_INFO(get_logger(), "6Dof Marker created and added to server.");
  }

  void BasicControlsNode::frameCallback()
  {
    static uint32_t counter = 0;

    if (!tf_broadcaster_)
    {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
    }

    geometry_msgs::msg::TransformStamped transform_msg;

    // First transform: "base_link" -> "moving_frame"
    transform_msg.header.stamp = this->get_clock()->now();
    transform_msg.header.frame_id = "base_link";
    transform_msg.child_frame_id = "moving_frame";
    transform_msg.transform.translation.x = 0.0;
    transform_msg.transform.translation.y = 0.0;
    transform_msg.transform.translation.z = sin(static_cast<double>(counter) / 140.0) * 2.0;
    transform_msg.transform.rotation.w = 1.0;
    transform_msg.transform.rotation.x = 0.0;
    transform_msg.transform.rotation.y = 0.0;
    transform_msg.transform.rotation.z = 0.0;

    RCLCPP_INFO(get_logger(), "Broadcasting TF: %s -> %s",
                transform_msg.header.frame_id.c_str(), transform_msg.child_frame_id.c_str());

    tf_broadcaster_->sendTransform(transform_msg);

    // Second transform: "base_link" -> "rotating_frame"
    tf2::Quaternion quat;
    quat.setRPY(0.0, static_cast<double>(counter) / 140.0, 0.0);
    transform_msg.header.stamp = this->get_clock()->now();
    transform_msg.header.frame_id = "base_link";
    transform_msg.child_frame_id = "rotating_frame";
    transform_msg.transform.translation.x = 0.0;
    transform_msg.transform.translation.y = 0.0;
    transform_msg.transform.translation.z = 0.0;
    transform_msg.transform.rotation = tf2::toMsg(quat);

    tf_broadcaster_->sendTransform(transform_msg);

    counter++;
  }
}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(interactive_marker_tutorials::BasicControlsPanel, rviz_common::Panel)

/// cube insi height dfkjkdj 

// //debugger put this line in  BasicControlsNode::BasicControlsNode(const rclcpp::NodeOptions &options)
//       : rclcpp::Node("basic_controls", options), menu_handler_()
//   {

//     this->get_logger().set_level(rclcpp::Logger::Level::Debug);
