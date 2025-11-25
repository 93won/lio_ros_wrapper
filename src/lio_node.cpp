/**
 * @file      lio_node.cpp
 * @brief     ROS2 wrapper node for LiDAR-Inertial Odometry
 * @author    Seungwon Choi
 * @email     csw3575@snu.ac.kr
 * @date      2025-11-22
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <vector>
#include <thread>
#include <atomic>

// LIO includes
#include "Estimator.h"
#include "ConfigUtils.h"
#include "SensorEventQueue.h"

using namespace lio;

/**
 * @brief LIO processing result for publisher thread
 */
struct LIOProcessingResult {
    bool success;
    double timestamp;
    State state;  // LIO state (position, rotation, velocity, biases)
    PointCloudPtr processed_cloud;  // Current scan (downsampled)
    PointCloudPtr raw_cloud;  // Raw scan (for visualization)
    
    LIOProcessingResult() : success(false), timestamp(0.0) {}
};

class LIONode : public rclcpp::Node
{
public:
    LIONode() : Node("lio_node"), 
                imu_count_(0),
                lidar_count_(0),
                running_(true),
                gravity_initialized_(false)
    {
        // Declare parameters
        this->declare_parameter<std::string>("imu_topic", "/livox/imu");
        this->declare_parameter<std::string>("lidar_topic", "/livox/lidar");
        this->declare_parameter<std::string>("config_file", "");
        this->declare_parameter<int>("init_imu_samples", 200);

        // Get parameters
        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
        std::string config_path = this->get_parameter("config_file").as_string();
        init_imu_samples_ = this->get_parameter("init_imu_samples").as_int();

        RCLCPP_INFO(this->get_logger(), "Starting LIO Node");
        RCLCPP_INFO(this->get_logger(), "IMU topic: %s", imu_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "LiDAR topic: %s", lidar_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Config file: %s", config_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Init IMU samples: %d", init_imu_samples_);

        // Load LIO configuration
        try {
            if (config_path.empty()) {
                throw std::runtime_error("Config file path not specified! Please set 'config_file' parameter.");
            }
            
            if (!LoadConfig(config_path, config_)) {
                throw std::runtime_error("Failed to load config file: " + config_path);
            }
            
            PrintConfig(config_);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load config: %s", e.what());
            throw;
        }

        // Create subscribers
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 1000,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                this->imuCallback(msg);
            });

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic, 10,
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->lidarCallback(msg);
            });

        // Create publishers
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/lio/odometry", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/lio/pose", 100);  // 100Hz for IMU rate
        current_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lio/current_scan", 10);
        raw_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lio/raw_scan", 10);
        map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lio/map", 10);
        trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("/lio/trajectory", 10);

        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Initialize estimator (will be done after gravity initialization)
        estimator_ = std::make_shared<Estimator>();
        
        // Configure estimator parameters from config
        estimator_->m_params.voxel_size = config_.estimator.voxel_size;
        estimator_->m_params.max_correspondences = config_.estimator.max_correspondences;
        estimator_->m_params.max_correspondence_distance = config_.estimator.max_correspondence_distance;
        estimator_->m_params.max_iterations = config_.estimator.max_iterations;
        estimator_->m_params.convergence_threshold = config_.estimator.convergence_threshold;
        estimator_->m_params.enable_undistortion = config_.estimator.enable_undistortion;
        estimator_->m_params.max_map_distance = config_.estimator.max_distance;
        estimator_->m_params.max_voxel_hit_count = config_.estimator.max_voxel_hit_count;
        estimator_->m_params.voxel_hierarchy_factor = config_.estimator.voxel_hierarchy_factor;
        estimator_->m_params.frustum_fov_horizontal = config_.estimator.frustum_fov_horizontal;
        estimator_->m_params.frustum_fov_vertical = config_.estimator.frustum_fov_vertical;
        estimator_->m_params.frustum_max_range = config_.estimator.frustum_max_range;
        estimator_->m_params.keyframe_translation_threshold = config_.estimator.keyframe_translation_threshold;
        estimator_->m_params.keyframe_rotation_threshold = config_.estimator.keyframe_rotation_threshold;
        estimator_->m_params.scan_planarity_threshold = config_.estimator.scan_planarity_threshold;
        estimator_->m_params.map_planarity_threshold = config_.estimator.map_planarity_threshold;
        
        // Configure IMU noise parameters
        estimator_->m_params.gyr_noise_std = config_.imu.gyro_noise_density;
        estimator_->m_params.acc_noise_std = config_.imu.acc_noise_density;
        estimator_->m_params.gyr_bias_noise_std = config_.imu.gyro_bias_random_walk;
        estimator_->m_params.acc_bias_noise_std = config_.imu.acc_bias_random_walk;
        
        // Configure extrinsics
        estimator_->m_params.R_il = config_.extrinsics.R_il.cast<float>();
        estimator_->m_params.t_il = config_.extrinsics.t_il.cast<float>();
        
        // Configure gravity
        estimator_->m_params.gravity = config_.imu.gravity.cast<float>();
        
        // Update process noise matrix
        estimator_->UpdateProcessNoise();
        
        RCLCPP_INFO(this->get_logger(), "[Estimator] Configured from YAML");
        RCLCPP_INFO(this->get_logger(), "  Voxel size: %.2f m", estimator_->m_params.voxel_size);
        RCLCPP_INFO(this->get_logger(), "  IMU noise: gyr=%.4f rad/s, acc=%.4f m/s²",
                    estimator_->m_params.gyr_noise_std,
                    estimator_->m_params.acc_noise_std);

        // Start worker threads
        processing_thread_ = std::thread(&LIONode::processingThreadLoop, this);
        publisher_thread_ = std::thread(&LIONode::publisherThreadLoop, this);

        RCLCPP_INFO(this->get_logger(), "LIO Node initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Processing & Publisher threads started");
        RCLCPP_INFO(this->get_logger(), "Waiting for %d IMU samples for gravity initialization...", 
                    init_imu_samples_);
    }
    
    ~LIONode() {
        RCLCPP_INFO(this->get_logger(), "Shutting down LIO Node...");
        
        // Stop threads
        running_ = false;
        event_queue_.shutdown();
        result_queue_.shutdown();
        
        // Wait for threads to finish
        if (processing_thread_.joinable()) {
            processing_thread_.join();
            RCLCPP_INFO(this->get_logger(), "Processing thread stopped");
        }
        
        if (publisher_thread_.joinable()) {
            publisher_thread_.join();
            RCLCPP_INFO(this->get_logger(), "Publisher thread stopped");
        }
        
        RCLCPP_INFO(this->get_logger(), "LIO Node shutdown complete");
    }

private:
    // IMU Callback: Lightweight - just push to queue
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_count_++;
        
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        
        // Create LIO IMUData
        auto imu_data = std::make_shared<IMUData>(
            timestamp,
            Eigen::Vector3f(msg->linear_acceleration.x,
                           msg->linear_acceleration.y,
                           msg->linear_acceleration.z),
            Eigen::Vector3f(msg->angular_velocity.x,
                           msg->angular_velocity.y,
                           msg->angular_velocity.z)
        );
        
        // Gravity initialization phase: collect IMU samples
        if (!gravity_initialized_) {
            std::lock_guard<std::mutex> lock(init_mutex_);
            init_imu_buffer_.push_back(*imu_data);
            
            if (init_imu_buffer_.size() >= static_cast<size_t>(init_imu_samples_)) {
                // Perform gravity initialization
                RCLCPP_INFO(this->get_logger(), 
                           "🔧 Starting gravity initialization with %zu IMU samples...", 
                           init_imu_buffer_.size());
                
                if (estimator_->GravityInitialization(init_imu_buffer_)) {
                    gravity_initialized_ = true;
                    RCLCPP_INFO(this->get_logger(), "Gravity initialization successful!");
                    RCLCPP_INFO(this->get_logger(), "LIO estimator is now ready to process data");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Gravity initialization failed!");
                    init_imu_buffer_.clear();  // Reset and try again
                }
            } else {
                // Progress update every 50 samples
                if (init_imu_buffer_.size() % 50 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Collecting IMU samples: %zu/%d", 
                               init_imu_buffer_.size(), init_imu_samples_);
                }
            }
            return;  // Don't push to queue until initialized
        }
        
        // Push to event queue (automatically sorted by timestamp)
        event_queue_.pushIMU(timestamp, imu_data);
        
        RCLCPP_DEBUG(this->get_logger(), "IMU #%ld @ %.6f - Queue size: %zu", 
                    imu_count_, timestamp, event_queue_.size());
    }
    
    // LiDAR Callback: Lightweight - just convert and push to queue
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!gravity_initialized_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                "Waiting for gravity initialization before processing LiDAR data...");
            return;  // Skip LiDAR until initialized
        }
        
        lidar_count_++;
        
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        
        // Convert ROS PointCloud2 to LIO PointCloud
        auto cloud = std::make_shared<PointCloud>();
        cloud->reserve(msg->width * msg->height);
        
        // Create iterators for x, y, z, intensity, offset_time
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        
        // Check if intensity field exists
        bool has_intensity = false;
        for (const auto& field : msg->fields) {
            if (field.name == "intensity") {
                has_intensity = true;
                break;
            }
        }
        
        // Check if offset_time field exists
        bool has_offset_time = false;
        for (const auto& field : msg->fields) {
            if (field.name == "offset_time") {
                has_offset_time = true;
                break;
            }
        }
        
        if (has_intensity && has_offset_time) {
            sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
            sensor_msgs::PointCloud2ConstIterator<uint32_t> iter_offset_time(*msg, "offset_time");
            
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_offset_time) {
                Point3D point;
                point.x = *iter_x;
                point.y = *iter_y;
                point.z = *iter_z;
                point.intensity = *iter_intensity;
                point.offset_time = static_cast<float>(static_cast<double>(*iter_offset_time) / 1e9);  // uint32 (ns) -> double -> float (sec)
                cloud->push_back(point);
            }
        } else if (has_intensity) {
            sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
            
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
                Point3D point;
                point.x = *iter_x;
                point.y = *iter_y;
                point.z = *iter_z;
                point.intensity = *iter_intensity;
                point.offset_time = 0.0f;  // No offset time
                cloud->push_back(point);
            }
        } else {
            // No intensity, no offset_time
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                Point3D point;
                point.x = *iter_x;
                point.y = *iter_y;
                point.z = *iter_z;
                point.intensity = 0.0f;
                point.offset_time = 0.0f;
                cloud->push_back(point);
            }
        }
        
        // Create LidarData
        auto lidar_data = std::make_shared<LidarData>(timestamp, cloud);
        
        // Push to event queue (automatically sorted by timestamp)
        event_queue_.pushLiDAR(timestamp, lidar_data);
        
        RCLCPP_DEBUG(this->get_logger(), "LiDAR #%ld @ %.6f - Points: %zu, Queue size: %zu", 
                    lidar_count_, timestamp, cloud->size(), event_queue_.size());
        
        size_t queue_size = event_queue_.size();
        if (queue_size > 100) {
            RCLCPP_WARN(this->get_logger(), 
                       "Event queue size: %zu (processing may be slower than incoming data)",
                       queue_size);
        }
    }
    
    // Processing Thread: Pop events in timestamp order and process
    void processingThreadLoop() {
        RCLCPP_INFO(this->get_logger(), "Processing thread started");
        
        size_t processed_count = 0;
        size_t lidar_processed = 0;
        size_t imu_processed = 0;
        
        double last_timestamp = -1.0;  // For timestamp ordering check
        
        while (running_) {
            // Pop oldest event (blocking)
            auto event_opt = event_queue_.pop();
            
            if (!event_opt.has_value()) {
                break;  // Queue shutdown
            }
            
            SensorEvent event = event_opt.value();
            processed_count++;
            
            // Check timestamp ordering (only warn if violation > 10ms)
            if (last_timestamp >= 0.0 && event.timestamp < last_timestamp) {
                double diff = event.timestamp - last_timestamp;
                if (diff < -0.01) {  // -10ms threshold
                    RCLCPP_ERROR(this->get_logger(), 
                                "TIMESTAMP ORDER VIOLATION! last=%.9f, current=%.9f (diff=%.9f)",
                                last_timestamp, event.timestamp, diff);
                } else {
                    RCLCPP_DEBUG(this->get_logger(), 
                                "Minor timestamp reordering: last=%.9f, current=%.9f (diff=%.9f)",
                                last_timestamp, event.timestamp, diff);
                }
            }
            last_timestamp = event.timestamp;
            
            try {
                if (event.type == SensorType::IMU) {
                    // Process IMU (propagate state)
                    estimator_->ProcessIMU(*event.imu_data);
                    imu_processed++;
                    
                    // Publish pose at IMU rate (100Hz)
                    publishPoseOnly(estimator_->GetCurrentState(), event.timestamp);
                    
                    RCLCPP_DEBUG(this->get_logger(), "[%zu] IMU @ %.9f | acc=[%.3f, %.3f, %.3f] gyr=[%.3f, %.3f, %.3f]", 
                                processed_count, event.timestamp,
                                event.imu_data->acc.x(),
                                event.imu_data->acc.y(),
                                event.imu_data->acc.z(),
                                event.imu_data->gyr.x(),
                                event.imu_data->gyr.y(),
                                event.imu_data->gyr.z());
                    
                } else {
                    // Process LiDAR (scan matching + update)
                    estimator_->ProcessLidar(*event.lidar_data);
                    lidar_processed++;
                    
                    RCLCPP_DEBUG(this->get_logger(), 
                                "[%zu] LiDAR @ %.9f | Points: %zu | Queue: %zu", 
                                processed_count, event.timestamp, event.lidar_data->cloud->size(),
                                event_queue_.size());
                    
                    // Get current state
                    State current_state = estimator_->GetCurrentState();
                    
                    // Publish pose immediately (if there are subscribers)
                    if (pose_pub_->get_subscription_count() > 0) {
                        publishPoseOnly(current_state, event.timestamp);
                    }
                    
                    // Publish raw scan immediately (if there are subscribers)
                    if (raw_scan_pub_->get_subscription_count() > 0) {
                        rclcpp::Time ros_time(static_cast<int64_t>(event.timestamp * 1e9));
                        publishRawScan(event.lidar_data->cloud, current_state, ros_time);
                    }
                    
                    // Prepare result for publisher thread (other visualizations)
                    LIOProcessingResult result;
                    result.success = true;
                    result.timestamp = event.timestamp;
                    result.state = current_state;
                    result.processed_cloud = estimator_->GetProcessedCloud();
                    result.raw_cloud = nullptr;  // Already published in processing thread
                    result_queue_.push(result);
                }
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), 
                            "Exception in LIO processing: %s", e.what());
            }
        }
        
        RCLCPP_INFO(this->get_logger(), 
                   "Processing thread stopped (processed %zu events: %zu IMU, %zu LiDAR)", 
                   processed_count, imu_processed, lidar_processed);
    }
    
    // Publisher Thread: Publish ROS topics
    void publisherThreadLoop() {
        RCLCPP_INFO(this->get_logger(), "Publisher thread started");
        
        size_t published_count = 0;
        
        while (running_) {
            auto result_opt = result_queue_.pop();
            
            if (!result_opt.has_value()) {
                break;  // Queue shutdown
            }
            
            LIOProcessingResult result = result_opt.value();
            
            if (result.success) {
                // Publish odometry & TF
                publishOdometry(result.state, result.timestamp);
                
                // Publish visualization
                publishVisualization(result);
                
                published_count++;
                
                RCLCPP_DEBUG(this->get_logger(), "Published frame #%zu", published_count);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), 
                   "Publisher thread stopped (published %zu frames)", 
                   published_count);
    }
    
    // Publish pose only (called at IMU rate ~100Hz)
    void publishPoseOnly(const State& state, double timestamp)
    {
        rclcpp::Time ros_time(static_cast<int64_t>(timestamp * 1e9));
        
        Eigen::Quaternionf q(state.m_rotation);
        q.normalize();
        
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = ros_time;
        pose_msg.header.frame_id = "map";
        
        pose_msg.pose.position.x = state.m_position.x();
        pose_msg.pose.position.y = state.m_position.y();
        pose_msg.pose.position.z = state.m_position.z();
        
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        
        pose_pub_->publish(pose_msg);
    }
    
    void publishOdometry(const State& state, double timestamp)
    {
        // Create ROS timestamp
        rclcpp::Time ros_time(static_cast<int64_t>(timestamp * 1e9));
        
        // Convert rotation matrix to quaternion
        Eigen::Quaternionf q(state.m_rotation);
        q.normalize();
        
        // Publish Odometry
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = ros_time;
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";
        
        odom_msg.pose.pose.position.x = state.m_position.x();
        odom_msg.pose.pose.position.y = state.m_position.y();
        odom_msg.pose.pose.position.z = state.m_position.z();
        
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        
        // Linear velocity (world frame)
        odom_msg.twist.twist.linear.x = state.m_velocity.x();
        odom_msg.twist.twist.linear.y = state.m_velocity.y();
        odom_msg.twist.twist.linear.z = state.m_velocity.z();
        
        // Angular velocity (world frame)
        // Transform gyroscope (body frame) to world frame: omega_world = R_wb * omega_body
        Eigen::Vector3f omega_world = state.m_rotation * state.m_gyro;
        odom_msg.twist.twist.angular.x = omega_world.x();
        odom_msg.twist.twist.angular.y = omega_world.y();
        odom_msg.twist.twist.angular.z = omega_world.z();
        
        odom_pub_->publish(odom_msg);
        
        // Broadcast TF: map -> base_link
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = ros_time;
        transform.header.frame_id = "map";
        transform.child_frame_id = "base_link";
        
        transform.transform.translation.x = state.m_position.x();
        transform.transform.translation.y = state.m_position.y();
        transform.transform.translation.z = state.m_position.z();
        
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(transform);
        
        // Add to trajectory
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = ros_time;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = state.m_position.x();
        pose_msg.pose.position.y = state.m_position.y();
        pose_msg.pose.position.z = state.m_position.z();
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        
        trajectory_.poses.push_back(pose_msg);
        trajectory_.header.stamp = ros_time;
        trajectory_.header.frame_id = "map";
    }
    
    void publishVisualization(const LIOProcessingResult& result)
    {
        rclcpp::Time ros_time(static_cast<int64_t>(result.timestamp * 1e9));
        
        // 1. Publish processed scan (GREEN - downsampled)
        if (result.processed_cloud && !result.processed_cloud->empty()) {
            publishCurrentScan(result.processed_cloud, result.state, ros_time);
        }
        
        // 2. Raw scan already published in processing thread
        
        // 3. Publish map (RED)
        auto map_cloud = estimator_->GetMapPointCloud();
        if (map_cloud && !map_cloud->empty()) {
            publishMapCloud(map_cloud, ros_time);
        }
        
        // 4. Publish trajectory
        trajectory_pub_->publish(trajectory_);
    }
    
    void publishCurrentScan(const PointCloudPtr& cloud, const State& state, const rclcpp::Time& timestamp)
    {
        // Transform cloud to world frame
        Eigen::Matrix4f T_wb = Eigen::Matrix4f::Identity();
        T_wb.block<3, 3>(0, 0) = state.m_rotation;
        T_wb.block<3, 1>(0, 3) = state.m_position;
        
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = timestamp;
        cloud_msg.header.frame_id = "map";
        cloud_msg.height = 1;
        cloud_msg.width = cloud->size();
        cloud_msg.is_dense = false;
        cloud_msg.is_bigendian = false;
        
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(cloud->size());
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");
        
        for (const auto& point : *cloud) {
            // Transform to world frame
            Eigen::Vector3f p_w = T_wb.block<3, 3>(0, 0) * Eigen::Vector3f(point.x, point.y, point.z) + T_wb.block<3, 1>(0, 3);
            
            *iter_x = p_w.x();
            *iter_y = p_w.y();
            *iter_z = p_w.z();
            *iter_r = 0;
            *iter_g = 255;  // Green
            *iter_b = 0;
            
            ++iter_x; ++iter_y; ++iter_z;
            ++iter_r; ++iter_g; ++iter_b;
        }
        
        current_scan_pub_->publish(cloud_msg);
    }
    
    void publishRawScan(const PointCloudPtr& cloud, const State& state, const rclcpp::Time& timestamp)
    {
        // Transform cloud to world frame
        Eigen::Matrix4f T_wb = Eigen::Matrix4f::Identity();
        T_wb.block<3, 3>(0, 0) = state.m_rotation;
        T_wb.block<3, 1>(0, 3) = state.m_position;
        
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = timestamp;
        cloud_msg.header.frame_id = "map";
        cloud_msg.height = 1;
        cloud_msg.width = cloud->size();
        cloud_msg.is_dense = false;
        cloud_msg.is_bigendian = false;
        
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgba");
        modifier.resize(cloud->size());
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_a(cloud_msg, "a");
        
        for (const auto& point : *cloud) {
            // Transform to world frame
            Eigen::Vector3f p_w = T_wb.block<3, 3>(0, 0) * Eigen::Vector3f(point.x, point.y, point.z) + T_wb.block<3, 1>(0, 3);
            
            *iter_x = p_w.x();
            *iter_y = p_w.y();
            *iter_z = p_w.z();
            *iter_r = 0;
            *iter_g = 0;
            *iter_b = 255;  // Blue (raw scan)
            *iter_a = 26;   // Alpha = 26/255 ≈ 0.1
            
            ++iter_x; ++iter_y; ++iter_z;
            ++iter_r; ++iter_g; ++iter_b; ++iter_a;
        }
        
        raw_scan_pub_->publish(cloud_msg);
    }
    
    void publishMapCloud(const PointCloudPtr& cloud, const rclcpp::Time& timestamp)
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = timestamp;
        cloud_msg.header.frame_id = "map";
        cloud_msg.height = 1;
        cloud_msg.width = cloud->size();
        cloud_msg.is_dense = false;
        cloud_msg.is_bigendian = false;
        
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(cloud->size());
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");
        
        for (const auto& point : *cloud) {
            *iter_x = point.x;
            *iter_y = point.y;
            *iter_z = point.z;
            *iter_r = 255;  // Red (map)
            *iter_g = 0;
            *iter_b = 0;
            
            ++iter_x; ++iter_y; ++iter_z;
            ++iter_r; ++iter_g; ++iter_b;
        }
        
        map_cloud_pub_->publish(cloud_msg);
    }

    // Thread-safe queue for results
    class ResultQueue {
    public:
        void push(const LIOProcessingResult& result) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                queue_.push(result);
            }
            cv_.notify_one();
        }
        
        std::optional<LIOProcessingResult> pop() {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this]() { return !queue_.empty() || shutdown_; });
            
            if (shutdown_ && queue_.empty()) {
                return std::nullopt;
            }
            
            LIOProcessingResult result = queue_.front();
            queue_.pop();
            return result;
        }
        
        size_t size() const {
            std::lock_guard<std::mutex> lock(mutex_);
            return queue_.size();
        }
        
        void shutdown() {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                shutdown_ = true;
            }
            cv_.notify_all();
        }
        
    private:
        std::queue<LIOProcessingResult> queue_;
        mutable std::mutex mutex_;
        std::condition_variable cv_;
        bool shutdown_ = false;
    };

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr current_scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    
    // TF broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Trajectory storage
    nav_msgs::msg::Path trajectory_;

    // LIO Estimator
    std::shared_ptr<Estimator> estimator_;
    
    // Configuration
    LIOConfig config_;
    int init_imu_samples_;
    
    // Gravity initialization
    bool gravity_initialized_;
    std::vector<IMUData> init_imu_buffer_;
    std::mutex init_mutex_;
    
    // Event queue (timestamp-sorted)
    SensorEventQueue event_queue_;
    ResultQueue result_queue_;
    
    // Worker threads
    std::thread processing_thread_;
    std::thread publisher_thread_;
    std::atomic<bool> running_;
    
    // Counters
    size_t imu_count_;
    size_t lidar_count_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LIONode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
