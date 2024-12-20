// Copyright 2022 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#define EIGEN_MPL2_ONLY

#include "obstacle_pointcloud_validator.hpp"

#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <object_recognition_utils/object_recognition_utils.hpp>

#include <boost/geometry.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace autoware::detected_object_validation
{
namespace obstacle_pointcloud
{
namespace bg = boost::geometry;
using Shape = autoware_perception_msgs::msg::Shape;
using Polygon2d = autoware::universe_utils::Polygon2d;

Validator::Validator(const PointsNumThresholdParam & points_num_threshold_param)
{
  points_num_threshold_param_.min_points_num = points_num_threshold_param.min_points_num;
  points_num_threshold_param_.max_points_num = points_num_threshold_param.max_points_num;
  points_num_threshold_param_.min_points_and_distance_ratio =
    points_num_threshold_param.min_points_and_distance_ratio;
}

size_t Validator::getThresholdPointCloud(
  const autoware_perception_msgs::msg::DetectedObject & object)
{
  const auto object_label_id = object.classification.front().label;
  const auto object_distance = std::hypot(
    object.kinematics.pose_with_covariance.pose.position.x,
    object.kinematics.pose_with_covariance.pose.position.y);
  size_t threshold_pc = std::clamp(
    static_cast<size_t>(
      points_num_threshold_param_.min_points_and_distance_ratio.at(object_label_id) /
        object_distance +
      0.5f),
    static_cast<size_t>(points_num_threshold_param_.min_points_num.at(object_label_id)),
    static_cast<size_t>(points_num_threshold_param_.max_points_num.at(object_label_id)));
  return threshold_pc;
}

Validator2D::Validator2D(PointsNumThresholdParam & points_num_threshold_param)
: Validator(points_num_threshold_param)
{
}

bool Validator2D::setKdtreeInputCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_cloud)
{
  obstacle_pointcloud_.reset(new pcl::PointCloud<pcl::PointXY>);
  pcl::fromROSMsg(*input_cloud, *obstacle_pointcloud_);
  if (obstacle_pointcloud_->empty()) {
    return false;
  }

  kdtree_ = pcl::make_shared<pcl::search::KdTree<pcl::PointXY>>(false);
  kdtree_->setInputCloud(obstacle_pointcloud_);

  return true;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr Validator2D::convertToXYZ(
  const pcl::PointCloud<pcl::PointXY>::Ptr & pointcloud_xy)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pointcloud_xyz->reserve(pointcloud_xy->size());
  for (const auto & point : *pointcloud_xy) {
    pointcloud_xyz->push_back(pcl::PointXYZ(point.x, point.y, 0.0));
  }
  return pointcloud_xyz;
}

std::optional<size_t> Validator2D::getPointCloudWithinObject(
  const autoware_perception_msgs::msg::DetectedObject & object,
  const pcl::PointCloud<pcl::PointXY>::Ptr pointcloud)
{
  std::vector<pcl::Vertices> vertices_array;
  pcl::Vertices vertices;
  Polygon2d poly2d = autoware::universe_utils::toPolygon2d(
    object.kinematics.pose_with_covariance.pose, object.shape);
  if (bg::is_empty(poly2d)) return std::nullopt;

  pcl::PointCloud<pcl::PointXYZ>::Ptr poly3d(new pcl::PointCloud<pcl::PointXYZ>);

  for (size_t i = 0; i < poly2d.outer().size(); ++i) {
    vertices.vertices.emplace_back(i);
    vertices_array.emplace_back(vertices);
    poly3d->emplace_back(poly2d.outer().at(i).x(), poly2d.outer().at(i).y(), 0.0);
  }
  cropped_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::CropHull<pcl::PointXYZ> cropper;  // don't be implemented PointXY by PCL
  cropper.setInputCloud(convertToXYZ(pointcloud));
  cropper.setDim(2);
  cropper.setHullIndices(vertices_array);
  cropper.setHullCloud(poly3d);
  cropper.setCropOutside(true);
  cropper.filter(*cropped_pointcloud_);
  return cropped_pointcloud_->size();
}

bool Validator2D::validate_object(
  const autoware_perception_msgs::msg::DetectedObject & transformed_object)
{
  // get neighbor_pointcloud of object
  neighbor_pointcloud_.reset(new pcl::PointCloud<pcl::PointXY>);
  std::vector<int> indices;
  std::vector<float> distances;
  const auto search_radius = getMaxRadius(transformed_object);
  if (!search_radius) {
    return false;
  }
  kdtree_->radiusSearch(
    pcl::PointXY(
      transformed_object.kinematics.pose_with_covariance.pose.position.x,
      transformed_object.kinematics.pose_with_covariance.pose.position.y),
    search_radius.value(), indices, distances);
  for (const auto & index : indices) {
    neighbor_pointcloud_->push_back(obstacle_pointcloud_->at(index));
  }
  const auto num = getPointCloudWithinObject(transformed_object, neighbor_pointcloud_);
  if (!num) return true;

  size_t threshold_pointcloud_num = getThresholdPointCloud(transformed_object);
  if (num.value() > threshold_pointcloud_num) {
    return true;
  }
  return false;  // remove object
}

std::optional<float> Validator2D::getMaxRadius(
  const autoware_perception_msgs::msg::DetectedObject & object)
{
  if (object.shape.type == Shape::BOUNDING_BOX || object.shape.type == Shape::CYLINDER) {
    return std::hypot(object.shape.dimensions.x * 0.5f, object.shape.dimensions.y * 0.5f);
  } else if (object.shape.type == Shape::POLYGON) {
    float max_dist = 0.0;
    for (const auto & point : object.shape.footprint.points) {
      const float dist = std::hypot(point.x, point.y);
      max_dist = max_dist < dist ? dist : max_dist;
    }
    return max_dist;
  } else {
    return std::nullopt;
  }
}

Validator3D::Validator3D(PointsNumThresholdParam & points_num_threshold_param)
: Validator(points_num_threshold_param)
{
}
bool Validator3D::setKdtreeInputCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_cloud)
{
  obstacle_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_cloud, *obstacle_pointcloud_);
  if (obstacle_pointcloud_->empty()) {
    return false;
  }
  // setup kdtree_
  kdtree_ = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>(false);
  kdtree_->setInputCloud(obstacle_pointcloud_);
  return true;
}
std::optional<float> Validator3D::getMaxRadius(
  const autoware_perception_msgs::msg::DetectedObject & object)
{
  if (object.shape.type == Shape::BOUNDING_BOX || object.shape.type == Shape::CYLINDER) {
    auto square_radius = (object.shape.dimensions.x * 0.5f) * (object.shape.dimensions.x * 0.5f) +
                         (object.shape.dimensions.y * 0.5f) * (object.shape.dimensions.y * 0.5f) +
                         (object.shape.dimensions.z * 0.5f) * (object.shape.dimensions.z * 0.5f);
    return std::sqrt(square_radius);
  } else if (object.shape.type == Shape::POLYGON) {
    float max_dist = 0.0;
    for (const auto & point : object.shape.footprint.points) {
      const float dist = std::hypot(point.x, point.y);
      max_dist = max_dist < dist ? dist : max_dist;
    }
    return std::hypot(max_dist, object.shape.dimensions.z * 0.5f);
  } else {
    return std::nullopt;
  }
}

std::optional<size_t> Validator3D::getPointCloudWithinObject(
  const autoware_perception_msgs::msg::DetectedObject & object,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr neighbor_pointcloud)
{
  std::vector<pcl::Vertices> vertices_array;
  pcl::Vertices vertices;
  auto const & object_position = object.kinematics.pose_with_covariance.pose.position;
  auto const object_height = object.shape.dimensions.x;
  auto z_min = object_position.z - object_height / 2.0f;
  auto z_max = object_position.z + object_height / 2.0f;
  Polygon2d poly2d = autoware::universe_utils::toPolygon2d(
    object.kinematics.pose_with_covariance.pose, object.shape);
  if (bg::is_empty(poly2d)) return std::nullopt;

  pcl::PointCloud<pcl::PointXYZ>::Ptr poly3d(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < poly2d.outer().size(); ++i) {
    vertices.vertices.emplace_back(i);
    vertices_array.emplace_back(vertices);
    poly3d->emplace_back(poly2d.outer().at(i).x(), poly2d.outer().at(i).y(), 0.0);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_pointcloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::CropHull<pcl::PointXYZ> cropper;
  cropper.setInputCloud(neighbor_pointcloud);
  cropper.setDim(2);
  cropper.setHullIndices(vertices_array);
  cropper.setHullCloud(poly3d);
  cropper.setCropOutside(true);
  cropper.filter(*cropped_pointcloud_2d);

  cropped_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  cropped_pointcloud_->reserve(cropped_pointcloud_2d->size());
  for (const auto & point : *cropped_pointcloud_2d) {
    if (point.z > z_min && point.z < z_max) {
      cropped_pointcloud_->push_back(point);
    }
  }
  return cropped_pointcloud_->size();
}

bool Validator3D::validate_object(
  const autoware_perception_msgs::msg::DetectedObject & transformed_object)
{
  neighbor_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> indices;
  std::vector<float> distances;
  const auto search_radius = getMaxRadius(transformed_object);
  if (!search_radius) {
    return false;
  }
  kdtree_->radiusSearch(
    pcl::PointXYZ(
      transformed_object.kinematics.pose_with_covariance.pose.position.x,
      transformed_object.kinematics.pose_with_covariance.pose.position.y,
      transformed_object.kinematics.pose_with_covariance.pose.position.z),
    search_radius.value(), indices, distances);
  for (const auto & index : indices) {
    neighbor_pointcloud_->push_back(obstacle_pointcloud_->at(index));
  }

  const auto num = getPointCloudWithinObject(transformed_object, neighbor_pointcloud_);
  if (!num) return true;

  size_t threshold_pointcloud_num = getThresholdPointCloud(transformed_object);
  if (num.value() > threshold_pointcloud_num) {
    return true;
  }
  return false;  // remove object
}

ObstaclePointCloudBasedValidator::ObstaclePointCloudBasedValidator(
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node("obstacle_pointcloud_based_validator", node_options),
  obstacle_pointcloud_sub_(
    this, "~/input/obstacle_pointcloud",
    rclcpp::SensorDataQoS{}.keep_last(1).get_rmw_qos_profile()),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  points_num_threshold_param_.min_points_num =
    declare_parameter<std::vector<int64_t>>("min_points_num");
  points_num_threshold_param_.max_points_num =
    declare_parameter<std::vector<int64_t>>("max_points_num");
  points_num_threshold_param_.min_points_and_distance_ratio =
    declare_parameter<std::vector<double>>("min_points_and_distance_ratio");

  using_2d_validator_ = declare_parameter<bool>("using_2d_validator");
  lidar_detection_model_ = declare_parameter("lidar_detection_model", "centerpoint");

  using std::placeholders::_1;
  using std::placeholders::_2;

  if(lidar_detection_model_ == "valor"){
    multiarr_sub_ = std::make_unique<message_filters::Subscriber<valo_msgs::msg::Float32MultiArrayStamped>>(
        this, "~/input/detected_objects", rclcpp::QoS{1}.get_rmw_qos_profile());
    sync_multiarr_ = std::make_unique<SyncMultiArr>(SyncPolicyMultiArr(10), *multiarr_sub_, obstacle_pointcloud_sub_);
    sync_multiarr_->registerCallback(
        std::bind(&ObstaclePointCloudBasedValidator::onMultiArrAndObstaclePointCloud, this, _1, _2));
  }
  else{
    objects_sub_ = std::make_unique<message_filters::Subscriber<autoware_perception_msgs::msg::DetectedObjects>>(
        this, "~/input/detected_objects", rclcpp::QoS{1}.get_rmw_qos_profile());
    sync_detobj_ = std::make_unique<SyncDetObj>(SyncPolicyDetObj(10), *objects_sub_, obstacle_pointcloud_sub_);
    sync_detobj_->registerCallback(
        std::bind(&ObstaclePointCloudBasedValidator::onObjectsAndObstaclePointCloud, this, _1, _2));
  }

  if (using_2d_validator_) {
    validator_ = std::make_unique<Validator2D>(points_num_threshold_param_);
  } else {
    validator_ = std::make_unique<Validator3D>(points_num_threshold_param_);
  }

  objects_pub_ = create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "~/output/objects", rclcpp::QoS{1});
  debug_publisher_ = std::make_unique<autoware::universe_utils::DebugPublisher>(
    this, "obstacle_pointcloud_based_validator");

  const bool enable_debugger = declare_parameter<bool>("enable_debugger");
  if (enable_debugger) debugger_ = std::make_shared<Debugger>(this);
  published_time_publisher_ =
    std::make_unique<autoware::universe_utils::PublishedTimePublisher>(this);
}
void ObstaclePointCloudBasedValidator::onObjectsAndObstaclePointCloud(
  const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr & input_objects,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_obstacle_pointcloud)
{
  autoware_perception_msgs::msg::DetectedObjects output, removed_objects;
  output.header = input_objects->header;
  removed_objects.header = input_objects->header;

  // Transform to pointcloud frame
  autoware_perception_msgs::msg::DetectedObjects transformed_objects;
  if (!object_recognition_utils::transformObjects(
        *input_objects, input_obstacle_pointcloud->header.frame_id, tf_buffer_,
        transformed_objects)) {
    // objects_pub_->publish(*input_objects);
    return;
  }
  bool validation_is_ready = true;
  if (!validator_->setKdtreeInputCloud(input_obstacle_pointcloud)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5,
      "obstacle pointcloud is empty! Can not validate objects.");
    validation_is_ready = false;
  }

  for (size_t i = 0; i < transformed_objects.objects.size(); ++i) {
    const auto & transformed_object = transformed_objects.objects.at(i);
    const auto & object = input_objects->objects.at(i);
    const auto validated =
      validation_is_ready ? validator_->validate_object(transformed_object) : false;
    if (debugger_) {
      debugger_->addNeighborPointcloud(validator_->getDebugNeighborPointCloud());
      debugger_->addPointcloudWithinPolygon(validator_->getDebugPointCloudWithinObject());
    }
    if (validated) {
      output.objects.push_back(object);
    } else {
      removed_objects.objects.push_back(object);
    }
  }

  objects_pub_->publish(output);
  published_time_publisher_->publish_if_subscribed(objects_pub_, output.header.stamp);
  if (debugger_) {
    debugger_->publishRemovedObjects(removed_objects);
    debugger_->publishNeighborPointcloud(input_obstacle_pointcloud->header);
    debugger_->publishPointcloudWithinPolygon(input_obstacle_pointcloud->header);
  }

  // Publish processing time info
  const double pipeline_latency =
    std::chrono::duration<double, std::milli>(
      std::chrono::nanoseconds((this->get_clock()->now() - output.header.stamp).nanoseconds()))
      .count();
  debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
    "debug/pipeline_latency_ms", pipeline_latency);
}


autoware_perception_msgs::msg::DetectedObjects::SharedPtr ObstaclePointCloudBasedValidator::convertMultiArrToDetObjs(
		const valo_msgs::msg::Float32MultiArrayStamped::ConstSharedPtr & input_objects_raw_msgs)
{
	//11 elems: x y z dimx dimy dimz yaw velx vely score label
  //assert(input_objects_raw_msgs->array.layout.dim[1] == 11);
  using Label = autoware_perception_msgs::msg::ObjectClassification;
	const std::array<int, 6> cls_mapping = {-1, Label::CAR, Label::TRUCK, Label::BUS, Label::BICYCLE, Label::PEDESTRIAN}; // Is it correct?

	auto detected_objs = std::make_shared<autoware_perception_msgs::msg::DetectedObjects>();
	detected_objs->header = input_objects_raw_msgs->header;

	const int num_objs = input_objects_raw_msgs->array.layout.dim[0].size;
	const int stride = input_objects_raw_msgs->array.layout.dim[1].stride;
	auto& data_vec = input_objects_raw_msgs->array.data;
	detected_objs->objects.resize(num_objs);

//    RCLCPP_INFO(rclcpp::get_logger("multi_object_tracker"), "Objects:\n");
	for(auto i=0; i<num_objs; ++i){
		auto& obj = detected_objs->objects[i];
		auto obj_idx = i*stride;

		obj.existence_probability = data_vec[obj_idx + 9];

		autoware_perception_msgs::msg::ObjectClassification classification;
		classification.probability = 1.0f;
		auto label_pcdet = static_cast<unsigned char>(data_vec[obj_idx + 10]);
		if (label_pcdet >= 1 && static_cast<size_t>(label_pcdet) < cls_mapping.size()) {
			classification.label = cls_mapping[label_pcdet];
		} else {
			classification.label = Label::UNKNOWN;
			RCLCPP_WARN_STREAM(this->get_logger(), "Unexpected label: UNKNOWN is set.");
		}

		obj.classification.emplace_back(classification);

		if (object_recognition_utils::isCarLikeVehicle(classification.label)) {
			obj.kinematics.orientation_availability =
				autoware_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
		}

		float yaw = data_vec[obj_idx + 6]; // - tier4_autoware_utils::pi / 2;
		obj.kinematics.pose_with_covariance.pose.position =
			autoware::universe_utils::createPoint(data_vec[obj_idx], data_vec[obj_idx + 1], data_vec[obj_idx + 2]);
		obj.kinematics.pose_with_covariance.pose.orientation =
			autoware::universe_utils::createQuaternionFromYaw(yaw);
		obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
		obj.shape.dimensions =
			autoware::universe_utils::createTranslation(data_vec[obj_idx + 3], data_vec[obj_idx + 4], data_vec[obj_idx + 5]);

		// twist
		float vel_x = data_vec[obj_idx + 7];
		float vel_y = data_vec[obj_idx + 8];
		geometry_msgs::msg::Twist twist;
		twist.linear.x = std::sqrt(std::pow(vel_x, 2) + std::pow(vel_y, 2));
		twist.angular.z = 2 * (std::atan2(vel_y, vel_x) - yaw);
		obj.kinematics.twist_with_covariance.twist = twist;
		obj.kinematics.has_twist = true;

			//Lets try doing the inverse calculation , just for debugging
//        auto rpy = tier4_autoware_utils::getRPY(obj.kinematics.pose_with_covariance.pose.orientation);
//        auto new_yaw = rpy.z; // this ir right
//        float vel_x_new = twist.linear.x * std::cos(new_yaw);
//        float vel_y_new = twist.linear.x * std::sin(new_yaw);
//        RCLCPP_INFO(rclcpp::get_logger("multi_object_tracker"), "yaw: %f %f vel_x: %f %f vel_y: %f %f\n", yaw, new_yaw, vel_x, vel_x_new, vel_y, vel_y_new);
	}
	return detected_objs;
}


void ObstaclePointCloudBasedValidator::onMultiArrAndObstaclePointCloud(
    const valo_msgs::msg::Float32MultiArrayStamped::ConstSharedPtr & input_objects,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_obstacle_pointcloud){

	auto detobjs = convertMultiArrToDetObjs(input_objects);
	onObjectsAndObstaclePointCloud(detobjs, input_obstacle_pointcloud);
}



}  // namespace obstacle_pointcloud
}  // namespace autoware::detected_object_validation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::detected_object_validation::obstacle_pointcloud::ObstaclePointCloudBasedValidator)
