/**
 * @file      SensorEventQueue.h
 * @brief     Thread-safe priority queue for sensor events with timestamp-based ordering
 * @author    Seungwon Choi
 * @email     csw3575@snu.ac.kr
 * @date      2025-11-22
 * @copyright Copyright (c) 2025 Seungwon Choi. All rights reserved.
 *
 * @par License
 * This project is released under the MIT License.
 */

#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <memory>
#include <Eigen/Core>

// Forward declaration for LIO types
namespace lio {
    struct IMUData;
    struct LidarData;
}

/**
 * @brief Sensor type enum
 */
enum class SensorType {
    IMU,
    LIDAR
};

/**
 * @brief Sensor event with timestamp-based ordering
 * lio_player.cpp의 SensorEvent 패턴을 ROS2 realtime에 적용
 */
struct SensorEvent {
    SensorType type;
    double timestamp;
    
    // Actual sensor data (using shared_ptr for efficiency)
    std::shared_ptr<lio::IMUData> imu_data;
    std::shared_ptr<lio::LidarData> lidar_data;
    
    SensorEvent() : type(SensorType::IMU), timestamp(0.0) {}
    
    SensorEvent(double ts, std::shared_ptr<lio::IMUData> imu)
        : type(SensorType::IMU), timestamp(ts), imu_data(imu), lidar_data(nullptr) {}
    
    SensorEvent(double ts, std::shared_ptr<lio::LidarData> lidar)
        : type(SensorType::LIDAR), timestamp(ts), imu_data(nullptr), lidar_data(lidar) {}
    
    // Comparison operator for priority queue (MIN heap - oldest timestamp first)
    bool operator<(const SensorEvent& other) const {
        // Reverse comparison for MIN heap (priority_queue is MAX heap by default)
        return timestamp > other.timestamp;
    }
};

/**
 * @brief Thread-safe priority queue for sensor events with timestamp-based ordering
 * 
 * lio_player.cpp의 CreateEventSequence() 패턴을 실시간으로 구현:
 * - IMU callback → push to queue
 * - LiDAR callback → push to queue
 * - Processing thread → pop in timestamp order (oldest first)
 * 
 * Key difference from lio_player:
 * - lio_player: Load all data, sort once, play back
 * - ROS2: Receive data realtime, maintain sorted order dynamically
 */
class SensorEventQueue {
public:
    SensorEventQueue() : shutdown_(false) {}
    
    ~SensorEventQueue() {
        shutdown();
    }
    
    /**
     * @brief Push IMU event to queue (automatically sorted by timestamp)
     */
    void pushIMU(double timestamp, std::shared_ptr<lio::IMUData> imu_data) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (shutdown_) return;
            
            SensorEvent event(timestamp, imu_data);
            queue_.push(event);
        }
        cv_.notify_one();
    }
    
    /**
     * @brief Push LiDAR event to queue (automatically sorted by timestamp)
     */
    void pushLiDAR(double timestamp, std::shared_ptr<lio::LidarData> lidar_data) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (shutdown_) return;
            
            SensorEvent event(timestamp, lidar_data);
            queue_.push(event);
        }
        cv_.notify_one();
    }
    
    /**
     * @brief Pop oldest event from queue (blocking)
     * @return std::optional<SensorEvent> The event, or std::nullopt if shutdown
     * 
     * ⭐ Key: Priority queue automatically gives us the OLDEST timestamp first!
     */
    std::optional<SensorEvent> pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        
        // Wait until queue has events or shutdown
        cv_.wait(lock, [this]() { 
            return !queue_.empty() || shutdown_; 
        });
        
        if (shutdown_ && queue_.empty()) {
            return std::nullopt;
        }
        
        // Get event with smallest timestamp (oldest)
        SensorEvent event = queue_.top();
        queue_.pop();
        return event;
    }
    
    /**
     * @brief Get current queue size
     */
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
    
    /**
     * @brief Check if queue is empty
     */
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
    
    /**
     * @brief Shutdown the queue (unblock all waiting threads)
     */
    void shutdown() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            shutdown_ = true;
        }
        cv_.notify_all();
    }
    
    /**
     * @brief Clear all events in the queue
     */
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        // Clear priority_queue (no clear() method, so recreate)
        std::priority_queue<SensorEvent> empty_queue;
        queue_.swap(empty_queue);
    }

private:
    // ⭐ Priority queue: automatically maintains timestamp ordering!
    std::priority_queue<SensorEvent> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool shutdown_;
};
