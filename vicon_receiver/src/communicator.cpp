#include "vicon_receiver/communicator.hpp"

using namespace ViconDataStreamSDK::CPP;

// Constructor for the Communicator class
Communicator::Communicator() : Node("vicon_client")
{
    // Declare parameters for hostname, buffer size, and namespace
    this->declare_parameter<std::string>("hostname", "127.0.0.1");
    this->declare_parameter<int>("buffer_size", 200);
    this->declare_parameter<std::string>("namespace", "vicon");
    this->declare_parameter<std::string>("world_frame", "map");
    this->declare_parameter<std::string>("vicon_frame", "vicon");
    this->declare_parameter<std::vector<double>>("map_xyz",  {0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("map_rpy",  {0.0, 0.0, 0.0});
    this->declare_parameter<bool>("map_rpy_in_degrees", false);

    // Retrieve parameters values
    this->get_parameter("hostname", hostname);
    this->get_parameter("buffer_size", buffer_size);
    this->get_parameter("namespace", ns_name);

    this->get_parameter("world_frame", world_frame);
    this->get_parameter("vicon_frame", vicon_frame);
    this->get_parameter("map_xyz", map_xyz);
    this->get_parameter("map_rpy", map_rpy);
    this->get_parameter("map_rpy_in_degrees", map_rpy_in_degrees);

    // Publish static transform from map ti vicon origin
    if (map_rpy_in_degrees) {
        for (unsigned int i=0; i<map_rpy.size(); i++) {
            map_rpy[i] = map_rpy[i] * M_PI / 180.0;
        }
    }

    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->publish_static_transform();

    // Initialize the tf2 broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

// Publish the static transform from map to vicon origin
void Communicator::publish_static_transform()
{
    static_tf.header.stamp = this->get_clock()->now();
    static_tf.header.frame_id = world_frame;
    static_tf.child_frame_id = vicon_frame;

    static_tf.transform.translation.x = map_xyz[0];
    static_tf.transform.translation.y = map_xyz[1];
    static_tf.transform.translation.z = map_xyz[2];
    tf2::Quaternion q;
    q.setRPY(
      map_rpy[0],
      map_rpy[1],
      map_rpy[2]
    );
    static_tf.transform.rotation.x = q.x();
    static_tf.transform.rotation.y = q.y();
    static_tf.transform.rotation.z = q.z();
    static_tf.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(static_tf);

    string msg = "Published static transform from " + world_frame + " to " + vicon_frame;
    cout << msg << endl;
}


// Connect to the Vicon server
bool Communicator::connect()
{
    // Log connection attempt
    string msg = "Connecting to " + hostname + " ...";
    cout << msg << endl;

    int counter = 0;
    // Retry connection until successful
    while (!vicon_client.IsConnected().Connected && rclcpp::ok())
    {   
        bool ok = (vicon_client.Connect(hostname).Result == Result::Success);
        if (!ok)
        {
            counter++;
            msg = "Connect failed, reconnecting (" + std::to_string(counter) + ")...";
            cout << msg << endl;
        }
    }
    if (!rclcpp::ok()) {
        std::cout << "Shutdown requested before connection established." << std::endl;
        return false;
    }

    // Log successful connection
    msg = "Connection successfully established with " + hostname;
    cout << msg << endl;

    // Enable various data streams from the Vicon server
    vicon_client.EnableSegmentData();
    vicon_client.EnableMarkerData();
    vicon_client.EnableUnlabeledMarkerData();
    vicon_client.EnableMarkerRayData();
    vicon_client.EnableDeviceData();
    vicon_client.EnableDebugData();

    // Set the stream mode and buffer size
    vicon_client.SetStreamMode(StreamMode::ClientPull);
    vicon_client.SetBufferSize(buffer_size);

    // Log initialization completion
    msg = "Initialization complete";
    cout << msg << endl;

    return true;
}

// Disconnect from the Vicon server
bool Communicator::disconnect()
{
    // If already disconnected, return true
    if (!vicon_client.IsConnected().Connected)
        return true;

    sleep(1); // Wait before disconnecting

    // Disable all data streams
    vicon_client.DisableSegmentData();
    vicon_client.DisableMarkerData();
    vicon_client.DisableUnlabeledMarkerData();
    vicon_client.DisableDeviceData();
    vicon_client.DisableCentroidData();

    // Log disconnection attempt
    string msg = "Disconnecting from " + hostname + "...";
    cout << msg << endl;

    // Disconnect from the server
    vicon_client.Disconnect();

    // Log successful disconnection
    msg = "Successfully disconnected";
    cout << msg << endl;

    // Verify disconnection
    return !vicon_client.IsConnected().Connected;
}

// Retrieve and process a frame of data from the Vicon server
void Communicator::get_frame()
{
    // Request a new frame
    vicon_client.GetFrame();
    Output_GetFrameNumber frame_number = vicon_client.GetFrameNumber();

    // Get the number of subjects in the frame
    unsigned int subject_count = vicon_client.GetSubjectCount().SubjectCount;

    map<string, Publisher>::iterator pub_it;

    // Iterate through each subject
    for (unsigned int subject_index = 0; subject_index < subject_count; ++subject_index)
    {
        // Get the subject name
        string subject_name = vicon_client.GetSubjectName(subject_index).SubjectName;

        // Get the number of segments for the subject
        unsigned int segment_count = vicon_client.GetSegmentCount(subject_name).SegmentCount;

        // Iterate through each segment
        for (unsigned int segment_index = 0; segment_index < segment_count; ++segment_index)
        {
            // Get the segment name
            string segment_name = vicon_client.GetSegmentName(subject_name, segment_index).SegmentName;

            // Retrieve the segment's global position and rotation
            Output_GetSegmentGlobalTranslation trans =
                vicon_client.GetSegmentGlobalTranslation(subject_name, segment_name);
            Output_GetSegmentGlobalRotationQuaternion quat =
                vicon_client.GetSegmentGlobalRotationQuaternion(subject_name, segment_name);


            // Build a TF message for this segment
            geometry_msgs::msg::TransformStamped tf_msg;

            // Use node clock to timestamp the transform
            tf_msg.header.stamp = this->get_clock()->now();

            // Parent and child frames: Vicon global origin -> subject_segment
            tf_msg.header.frame_id = vicon_frame;
            tf_msg.child_frame_id = subject_name + "_" + segment_name;

            // Vicon translations are in millimeters; convert to meters for ROS
            tf_msg.transform.translation.x = trans.Translation[0] / 1000.0;
            tf_msg.transform.translation.y = trans.Translation[1] / 1000.0;
            tf_msg.transform.translation.z = trans.Translation[2] / 1000.0;

            // Vicon quaternion order is [x, y, z, w]; copy directly
            tf_msg.transform.rotation.x = quat.Rotation[0];
            tf_msg.transform.rotation.y = quat.Rotation[1];
            tf_msg.transform.rotation.z = quat.Rotation[2];
            tf_msg.transform.rotation.w = quat.Rotation[3];

            // Publish the position data
            boost::mutex::scoped_try_lock lock(mutex);
            if (lock.owns_lock())
            {
                // Check if a publisher exists for the segment
                pub_it = pub_map.find(subject_name + "/" + segment_name);
                if (pub_it != pub_map.end())
                {
                    Publisher & pub = pub_it->second;

                    if (pub.is_ready)
                    {
                        // Build a PoseStamped in the Vicon frame from the computed TransformStamped.
                        geometry_msgs::msg::PoseStamped vicon_pose_msg;

                        // Header: copy timestamp and frame_id ("vicon") from the transform header.
                        vicon_pose_msg.header = tf_msg.header;

                        // Position: copy the already meter-converted translation components.
                        vicon_pose_msg.pose.position.x = tf_msg.transform.translation.x;
                        vicon_pose_msg.pose.position.y = tf_msg.transform.translation.y;
                        vicon_pose_msg.pose.position.z = tf_msg.transform.translation.z;

                        // Orientation: copy the quaternion (x, y, z, w) directly from the transform.
                        vicon_pose_msg.pose.orientation = tf_msg.transform.rotation;

                        // Transform the pose to the global frame
                        geometry_msgs::msg::PoseStamped global_pose_msg;
                        tf2::doTransform(vicon_pose_msg, global_pose_msg, static_tf);

                        // Publish the transformed pose
                        pub.publish(global_pose_msg);
                    }
                }
                else
                {
                    // Create a publisher if it doesn't exist, de-duplicating concurrent attempts
                    std::string key = subject_name + "/" + segment_name;
                    if (pending_publishers.find(key) == pending_publishers.end())
                    {
                        pending_publishers.insert(key);
                        lock.unlock();
                        create_publisher(subject_name, segment_name);
                    }
                    else
                    {
                        // Another thread is already creating this publisher
                        lock.unlock();
                    }
                }
            }

            // Broadcast the transform
            tf_broadcaster_->sendTransform(tf_msg);

        }
    }
}

// Create a publisher for a specific subject and segment
void Communicator::create_publisher(const string subject_name, const string segment_name)
{
    // Launch a thread to create the publisher
    boost::thread(&Communicator::create_publisher_thread, this, subject_name, segment_name);
}

// Thread function to create a publisher
void Communicator::create_publisher_thread(const string subject_name, const string segment_name)
{
    // Construct the topic name and key
    std::string topic_name = ns_name + "/" + subject_name + "/" + segment_name;
    std::string key = subject_name + "/" + segment_name;

    // Log publisher creation
    string msg = "Creating publisher for segment " + segment_name + " from subject " + subject_name;
    cout << msg << endl;

    // Create and store the publisher; then clear the pending flag
    boost::mutex::scoped_lock lock(mutex);
    pub_map.insert(std::map<std::string, Publisher>::value_type(key, Publisher(topic_name, this)));
    pending_publishers.erase(key);
    lock.unlock();
}

// Main function
int main(int argc, char** argv)
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Communicator>();

    // Connect to the Vicon server
    node->connect();

    // Continuously retrieve frames while ROS 2 is running
    while (rclcpp::ok()){
        node->get_frame();
    }

    // Disconnect from the Vicon server and shut down ROS 2
    node->disconnect();
    rclcpp::shutdown();
    return 0;
}