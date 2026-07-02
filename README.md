# WireAndPipeDetectionNode 参数配置表

## 1. 相机内参与畸变

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `fx` | double | 558.43329 | 内参矩阵水平焦距（像素） |
| `fy` | double | 540.11337 | 内参矩阵垂直焦距（像素） |
| `cx` | double | 301.78 | 主点横坐标（像素） |
| `cy` | double | 304.95 | 主点纵坐标（像素） |
| `camera_width` | int | 640 | 输入图像宽度 |
| `camera_height` | int | 480 | 输入图像高度 |
| `distortion_coefficients` | double[] | `[-0.567324, 0.177060, -0.056009, 0.016418, 0.0]` | 畸变系数 (k1, k2, p1, p2, k3)，全零则跳过去畸变 |

---

## 2. 相机安装位姿（单目测距核心）

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `camera_x` | double | 0.0 | 相对车体 X 向偏移（米） |
| `camera_y` | double | 0.0 | 相对车体 Y 向偏移（米） |
| `camera_z` | double | 0.905 | **离地高度（米）**，测距最敏感参数 |
| `camera_pitch` | double | 0.087266 | **俯仰角（弧度）**：正仰视，负俯视（常用前视下压为负） |
| `camera_yaw` | double | 0.0 | 偏航角（弧度） |
| `camera_roll` | double | 0.0 | 横滚角（弧度） |
| `h_fov_rad` | double | 1.5708 | 水平视场角（弧度），默认 90° |
| `v_fov_rad` | double | 1.1519 | 垂直视场角（弧度），默认 66° |

---

## 3. YOLO 检测与模型

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `yolo_model_path` | string | (包内默认路径) | ONNX 模型文件路径（绝对或相对 share 目录） |
| `class_names` | string[] | `["wire", "water_pipe"]` | 类别名称列表，顺序与模型输出对齐 |
| `wire_class_id` | int | 0 | `wire` 在 `class_names` 中的索引 |
| `water_pipe_class_id` | int | 1 | `water_pipe` 在 `class_names` 中的索引 |
| `conf_threshold` | double | 0.6 | 检测置信度阈值，低于此值丢弃 |
| `yolo_frame_skip` | int | 3 | 推理跳帧数（每 N 帧执行一次） |
| `filter_above_horizon` | bool | true | 是否过滤图像水平中线以上的目标（减少误检） |

---

## 4. 目标跟踪（Tracker）

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `confirm_threshold` | double | 1.0 | 累积对数几率得分超过此值才确认目标并参与避障 |
| `tracking_iou_threshold` | double | 0.3 | 帧间匹配 IoU 阈值 |
| `max_track_age` | int | 5 | 目标丢失后最大保留帧数，超时销毁 |
| `decay_factor` | double | 0.9 | 累积分数衰减因子（每帧乘以此值） |

---

## 5. 路径订阅与避障策略

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `global_plan_topic` | string | `teb_global_plan` | 全局路径话题（`nav_msgs/Path`） |
| `local_poses_topic` | string | `teb_poses` | 局部路径话题（`geometry_msgs/PoseArray`） |
| `global_search_distance` | double | 5.0 | 全局路径前向搜索距离（米） |
| `local_search_distance` | double | 5.0 | 局部路径前向搜索距离（米） |
| `pedestrian_distance_threshold` | double | 1.0 | 触发避障的路径‑障碍物距离阈值（米） |
| `avoid_hold_seconds` | double | 3.0 | 避障保持时间（秒），目标消失后仍保持警告 |

---

## 6. 可视化与调试

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `publish_annotated_image` | bool | true | 是否发布带标注的图像（`/annotated_image_wire`） |
| `publish_debug_map_markers` | bool | true | 是否发布调试标记（路径点、障碍物方块等） |
| `distance_log_throttle_sec` | double | 3.0 | 距离日志打印节流时间（秒） |

---

## 7. 通信与 TF

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `camera_topic` | string | `/rgb_camera_front/compressed` | 订阅的压缩图像话题 |
| `rgb_camera_frame` | string | `front_camera_color_frame` | 相机 TF 坐标帧 ID，用于转换到 `map` |
