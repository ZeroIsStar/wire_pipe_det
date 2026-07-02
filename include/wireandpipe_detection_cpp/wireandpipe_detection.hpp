#ifndef WIREANDPIPE_DETECTION_CPP__WIREANDPIPE_DETECTION_HPP_
#define WIREANDPIPE_DETECTION_CPP__WIREANDPIPE_DETECTION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/core.hpp>          // 确保包含 cv::Mat
#include <opencv2/calib3d.hpp>        // 可选，但建议包含以使用 cv::undistort（在源文件中已包含，头文件不需要也可）
#include <mutex>
#include <vector>
#include <memory>
#include <string>

// 定义检测框结构和跟踪项（如果头文件中未定义，请保留原有定义）
struct ObbBBox {
    float cx, cy;
    float width, height;
    float angle;
};

struct ObbTrackItem {
    int class_id;
    float confidence;
    ObbBBox bbox;
};

// 跟踪目标结构
struct TrackedTarget {
    int track_id;
    int class_id;
    ObbBBox bbox;
    float confidence;
    int age = 0;
    bool confirmed = false;
    float accumulated_score = 0.0;
    std::chrono::steady_clock::time_point last_update;
};

struct DetectionResult {
    bool detected = false;
    rclcpp::Time stamp;
    rclcpp::Time laser_stamp;
    std::string laser_frame_id;
    std::vector<geometry_msgs::msg::Point> obstacles_laser;
};

// 基类（如果未定义，需要定义）
class ObbYoloTracker {
public:
    virtual ~ObbYoloTracker() = default;
    virtual std::vector<ObbTrackItem> track(const cv::Mat &frame) = 0;
};

class WireAndPipeDetectionNode : public rclcpp::Node
{
public:
    WireAndPipeDetectionNode();

private:
    // ---- 回调函数 ----
    void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void globalPlanCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void localPosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void timerCallback();

    // ---- 工具函数 ----
    cv::Mat decodeCompressedImage(const sensor_msgs::msg::CompressedImage::SharedPtr &msg,
                                  rclcpp::Time &out_stamp);
    geometry_msgs::msg::Point estimate3DFromLaser(const ObbBBox &obb,
                                                  int img_width,
                                                  float &out_dist_laser,
                                                  std::string &out_laser_frame_id) const;
    bool runYoloTrack(const cv::Mat &frame, std::vector<ObbTrackItem> &tracks);
    void updateTracker(const std::vector<ObbTrackItem> &new_detections);
    std::vector<geometry_msgs::msg::Point> downsamplePath(
        const std::vector<geometry_msgs::msg::Point> &nav_points,
        double min_distance,
        double lookahead_distance);
    bool checkObstacleOnGlobalPath(
        const std::vector<geometry_msgs::msg::PointStamped> &obstacles,
        const std::vector<geometry_msgs::msg::Point> &downsampled_points,
        double threshold,
        double *hit_distance);
    bool checkObstacleOnLocalPath(
        const std::vector<geometry_msgs::msg::PointStamped> &obstacles,
        const std::vector<geometry_msgs::msg::Point> &downsampled_points,
        double threshold,
        double *hit_distance);
    double minDistanceToPath(const geometry_msgs::msg::Point &point,
                             const std::vector<geometry_msgs::msg::Point> &path) const;

    // ---- 成员变量 ----
    // 相机参数
    double fx_, fy_, cx_, cy_;
    int camera_width_, camera_height_;
    double h_fov_rad_, v_fov_rad_;
    double camera_x_, camera_y_, camera_z_, camera_pitch_, camera_yaw_, camera_roll_;
    double conf_threshold_;
    double distance_log_throttle_sec_;
    std::vector<std::string> class_names_;
    int wire_class_id_, water_pipe_class_id_;
    double global_search_distance_, local_search_distance_;
    double obstacle_distance_threshold_;
    double avoid_hold_seconds_;
    int yolo_frame_skip_;
    bool publish_annotated_image_;
    bool publish_debug_map_markers_;
    bool filter_above_horizon_;

    // 新增：畸变校正相关
    cv::Mat camera_matrix_;   // 相机内参矩阵
    cv::Mat dist_coeffs_;     // 畸变系数

    // 跟踪参数
    double confirm_threshold_;
    double tracking_iou_threshold_;
    int max_track_age_;
    double decay_factor_;
    int next_track_id_;
    std::vector<TrackedTarget> tracked_targets_;

    // 路径缓存
    nav_msgs::msg::Path::SharedPtr last_global_plan_;
    geometry_msgs::msg::PoseArray::SharedPtr last_local_poses_;
    bool global_plan_dirty_{false};
    bool local_poses_dirty_{false};
    std::vector<geometry_msgs::msg::Point> cached_ds_global_;
    std::vector<geometry_msgs::msg::Point> cached_ds_local_;
    mutable std::mutex path_cache_mutex_;

    // 检测结果
    DetectionResult detection_result_;
    mutable std::mutex detection_mutex_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // YOLO 检测器
    std::shared_ptr<ObbYoloTracker> yolo_;

    // 发布者/订阅者
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_camera_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_global_plan_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_local_poses_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_avoiding_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_image_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_obstacle_markers_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_downsampled_path_markers_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_raw_path_markers_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_wire_pipe_3d_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_map_markers_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_object_poses_;
    std::vector<geometry_msgs::msg::Point> cached_global_raw_;
    std::vector<geometry_msgs::msg::Point> cached_local_raw_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::CallbackGroup::SharedPtr global_plan_callback_group_;
    rclcpp::CallbackGroup::SharedPtr local_poses_callback_group_;

    // 状态标志
    bool last_trigger_state_{false};
    bool has_recent_detection_{false};
    rclcpp::Time last_obstacle_detect_time_;
    bool last_published_avoiding_{false};
    bool waiting_detect_log_printed_{false};
    bool first_obstacle_detected_logged_{false};
    bool last_global_match_{false};
    bool last_local_match_{false};
    uint64_t warning_event_id_{0};
    std::string detection_label_;
};

#endif  // WIREANDPIPE_DETECTION_CPP__WIREANDPIPE_DETECTION_HPP_