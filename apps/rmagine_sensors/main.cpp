
#include <chrono>
#include <memory>
#include <vector>
#include <random>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "tinyxml2.h"

#include <rmagine/types/sensor_models.h>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/types/Memory.hpp>
#include <rmagine/map/AssimpIO.hpp>
#include <rmagine/map/optix/OptixScene.hpp>
#include <rmagine/map/optix/OptixNode.hpp>
#include <rmagine/types/MemoryCuda.hpp>

using namespace rmagine;
using namespace tinyxml2;
using namespace std::chrono_literals;

inline Transform Convert(const geometry_msgs::msg::Pose& pose)
{
  Transform t;
  t.t.x = pose.position.x;
  t.t.y = pose.position.y;
  t.t.z = pose.position.z;
  t.R.x = pose.orientation.x;
  t.R.y = pose.orientation.y;
  t.R.z = pose.orientation.z;
  t.R.w = pose.orientation.w;
  return t;
}

inline geometry_msgs::msg::Pose Convert(const Transform& t)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = t.t.x;
  pose.position.y = t.t.y;
  pose.position.z = t.t.z;
  pose.orientation.x = t.R.x;
  pose.orientation.y = t.R.y;
  pose.orientation.z = t.R.z;
  pose.orientation.w = t.R.w;
  return pose;
}

void make_mjcf_tree(XMLElement* xml, OptixScenePtr scene, OptixNodePtr node, const std::filesystem::path& path)
{
  const char* name = xml->Attribute("name");
  const char* pos = xml->Attribute("pos");
  const char* quat = xml->Attribute("quat");
  if (name)
    node->name = name;
  Transform T = Transform::Identity();
  if (pos)
  {
    std::istringstream p(pos);
    p >> T.t.x >> T.t.y >> T.t.z;
  }
  if (quat)
  {
    std::istringstream q(quat);
    q >> T.R.w >> T.R.x >> T.R.y >> T.R.z;
  }
  node->setTransform(T);
  if(const char* mesh = xml->Attribute("mesh"))
  {
      node->set_mesh(scene->get_mesh(mesh));
  }
  for (XMLElement* body = xml->FirstChildElement(); body; body = body->NextSiblingElement())
  {
    const char* name = body->Name();
    if (strcmp(name,"include")==0)
    {
      std::filesystem::path file_path = path / body->Attribute("file");
      if (!std::filesystem::exists(file_path))
        file_path = path.parent_path() / body->Attribute("file");
      XMLDocument include_doc;
      include_doc.LoadFile(file_path.string().c_str());
      XMLElement* root = include_doc.RootElement();
      for (XMLElement* inner_body = root->FirstChildElement(); inner_body; inner_body = inner_body->NextSiblingElement())
      {
        const char* inner_name = inner_body->Name();
        if (strcmp(inner_name,"body")==0 || strcmp(inner_name,"geom")==0)
        {
          OptixNodePtr child = node->add_node();
          make_mjcf_tree(inner_body, scene, child, file_path.parent_path());
        }
      }
    }
    else if (strcmp(name,"body")==0 || strcmp(name,"geom")==0)
    {
      OptixNodePtr child = node->add_node();
      make_mjcf_tree(body, scene, child, path);
    }
  }
}

void recursive_add_mesh(XMLElement* assets, OptixScenePtr scene, const std::filesystem::path& path)
{
  AssimpIO io;
  for (XMLElement* asset = assets->FirstChildElement(); asset; asset = asset->NextSiblingElement())
  {
    const char* name = asset->Name();
    std::filesystem::path file_path = path / asset->Attribute("file");
    if (strcmp(name,"include")==0)
    {
      if (!std::filesystem::exists(file_path))
        file_path = path.parent_path() / asset->Attribute("file");
      XMLDocument include_doc;
      include_doc.LoadFile(file_path.string().c_str());
      XMLElement* root = include_doc.RootElement();
      recursive_add_mesh(root, scene, file_path.parent_path());
    }
    else if (strcmp(name,"mesh")==0)
    {
      if (!std::filesystem::exists(file_path))
        file_path = path.parent_path().parent_path() / "meshes" / asset->Attribute("file");
      const aiScene* ascene = io.ReadFile(file_path.string(), 0);
      OptixMeshPtr mesh = make_optix_mesh(ascene->mMeshes[0], scene->context());
      const char* scale = asset->Attribute("scale");
      if (scale)
      {
        std::istringstream s(scale);
        s >> mesh->local_scale.x >> mesh->local_scale.y >> mesh->local_scale.z;
      }
      mesh->name = asset->Attribute("name");
      //std::cout << mesh->name << ": " << mesh->vertices.size() << std::endl;
      mesh->commit();
      scene->add_mesh(mesh);
    }
  }
}

OptixMapPtr import_mjcf_map(const std::string& filepath, OptixContextPtr optix_ctx = optix_default_context())
{
  XMLDocument doc;
  doc.LoadFile(filepath.c_str());
  XMLElement* root = doc.RootElement();
  XMLElement* assets = root->FirstChildElement("asset");
  OptixScenePtr scene = std::make_shared<OptixScene>(optix_ctx);
  std::filesystem::path path = std::filesystem::path(filepath).parent_path();
  recursive_add_mesh(assets, scene, path);
  make_mjcf_tree(root->FirstChildElement("worldbody"), scene, scene->root(), path);
  scene->commit();
  return std::make_shared<OptixMap>(scene);
}

class OptixRosNode : public rclcpp::Node
{
public:
  OptixRosNode() : Node("OptixRosNode")
  {
    declare_parameter<std::string>("map_file");
    // 创建交互标记服务器
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "control_marker",
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_topics_interface(),
      this->get_node_services_interface());
    
    // 创建位姿发布者
    pub_cam = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "control_pose", 10);
    
    // 创建交互标记
    createInteractiveMarker();
    pub_lidar_front = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_front", 10);
    pub_lidar_back = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_back", 10);
    pub_img = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&OptixRosNode::timer_callback, this));

    std::string map_path = get_parameter("map_file").as_string();
    OptixMapPtr map = import_mjcf_map(map_path);
    OptixNodePtr base_link = map->scene()->root()->get_node("/base_link");
    lidar_front = base_link->add_lidar();
    lidar_front->create_sim(map);
    lidar_back = base_link->add_lidar();
    lidar_back->create_sim(map);
    RCLCPP_INFO(this->get_logger(), "PointCloud2 publisher started.");
  }

private:
  void createInteractiveMarker()
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";  // 参考坐标系
    int_marker.name = "control_point";
    int_marker.description = "可拖拽的控制点";
    int_marker.pose = Convert(CamT);
    int_marker.scale = 1.0;
    
    // 创建控制点标记（可视化的外观）
    visualization_msgs::msg::Marker box_marker;
    box_marker.type = visualization_msgs::msg::Marker::CUBE;
    box_marker.scale.x = 0.5;
    box_marker.scale.y = 0.5;
    box_marker.scale.z = 0.5;
    box_marker.color.r = 0.0;
    box_marker.color.g = 0.5;
    box_marker.color.b = 0.5;
    box_marker.color.a = 0.8;
    
    // 添加方向箭头
    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.scale.x = 1.0;
    arrow_marker.scale.y = 0.1;
    arrow_marker.scale.z = 0.1;
    arrow_marker.color.r = 1.0;
    arrow_marker.color.g = 0.0;
    arrow_marker.color.b = 0.0;
    arrow_marker.color.a = 1.0;
    
    // 创建位置控制（允许在3D空间中移动）
    // X轴移动
    visualization_msgs::msg::InteractiveMarkerControl move_control_x;
    move_control_x.name = "move_x";
    move_control_x.orientation.w = 1.0;
    move_control_x.orientation.x = 1.0;
    move_control_x.orientation.y = 0.0;
    move_control_x.orientation.z = 0.0;
    move_control_x.interaction_mode = 
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    move_control_x.always_visible = true;
    int_marker.controls.push_back(move_control_x);
    
    // Y轴移动
    visualization_msgs::msg::InteractiveMarkerControl move_control_y;
    move_control_y.name = "move_y";
    move_control_y.orientation.w = 1.0;
    move_control_y.orientation.x = 0.0;
    move_control_y.orientation.y = 0.0;
    move_control_y.orientation.z = 1.0;
    move_control_y.interaction_mode = 
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    move_control_y.always_visible = true;
    int_marker.controls.push_back(move_control_y);
    
    // Z轴移动
    visualization_msgs::msg::InteractiveMarkerControl move_control_z;
    move_control_z.name = "move_z";
    move_control_z.orientation.w = 1.0;
    move_control_z.orientation.x = 0.0;
    move_control_z.orientation.y = 1.0;
    move_control_z.orientation.z = 0.0;
    move_control_z.interaction_mode = 
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    move_control_z.always_visible = true;
    int_marker.controls.push_back(move_control_z);
    
    // 创建旋转控制（允许绕轴旋转）
    // X轴旋转
    visualization_msgs::msg::InteractiveMarkerControl rotate_control_x;
    rotate_control_x.name = "rotate_x";
    rotate_control_x.orientation.w = 1.0;
    rotate_control_x.orientation.x = 1.0;
    rotate_control_x.orientation.y = 0.0;
    rotate_control_x.orientation.z = 0.0;
    rotate_control_x.interaction_mode = 
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    rotate_control_x.always_visible = true;
    int_marker.controls.push_back(rotate_control_x);
    
    // Y轴旋转
    visualization_msgs::msg::InteractiveMarkerControl rotate_control_y;
    rotate_control_y.name = "rotate_y";
    rotate_control_y.orientation.w = 1.0;
    rotate_control_y.orientation.x = 0.0;
    rotate_control_y.orientation.y = 1.0;
    rotate_control_y.orientation.z = 0.0;
    rotate_control_y.interaction_mode = 
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    rotate_control_y.always_visible = true;
    int_marker.controls.push_back(rotate_control_y);
    
    // Z轴旋转
    visualization_msgs::msg::InteractiveMarkerControl rotate_control_z;
    rotate_control_z.name = "rotate_z";
    rotate_control_z.orientation.w = 1.0;
    rotate_control_z.orientation.x = 0.0;
    rotate_control_z.orientation.y = 0.0;
    rotate_control_z.orientation.z = 1.0;
    rotate_control_z.interaction_mode = 
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    rotate_control_z.always_visible = true;
    int_marker.controls.push_back(rotate_control_z);
    
    // 添加可视化标记
    visualization_msgs::msg::InteractiveMarkerControl visual_control;
    visual_control.name = "visualization";
    visual_control.interaction_mode = 
      visualization_msgs::msg::InteractiveMarkerControl::NONE;
    visual_control.always_visible = true;
    visual_control.markers.push_back(box_marker);
    visual_control.markers.push_back(arrow_marker);
    int_marker.controls.push_back(visual_control);
    
    // 将交互标记添加到服务器
    server_->insert(
      int_marker,
      std::bind(&OptixRosNode::markerFeedbackCallback, this, std::placeholders::_1));
    
    server_->applyChanges();
  }
  
  void markerFeedbackCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
  {
    if (feedback->event_type == 
        visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
    {
      // 更新当前位姿
      CamT = Convert(feedback->pose);
      
      RCLCPP_DEBUG(this->get_logger(),
        "位姿已更新: (%.2f, %.2f, %.2f)",
        feedback->pose.position.x,
        feedback->pose.position.y,
        feedback->pose.position.z);
      
      // 立即发布一次
      publishPose();
    }
  }
  
  void publishPose()
  {
    auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose_msg->header.stamp = this->now();
    pose_msg->header.frame_id = "map";
    pose_msg->pose = Convert(CamT);
    pub_cam->publish(*pose_msg);
  }

  void timer_callback()
  {
    // Transform T = Transform::Identity();
    // T.t.x = 0.0;
    // T.t.y = 0.0;
    // T.t.z = 1.0;
    publish_lidar(lidar_front, pub_lidar_front);
    publish_lidar(lidar_back, pub_lidar_back);
    //publish_camera(Tbm_gpu);
  }
  void publish_lidar(OptixLidarPtr lidar, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc)
  {
    using ResultT = Bundle<
    Points<VRAM_CUDA>
    // FaceIds<VRAM_CUDA>,
    // GeomIds<VRAM_CUDA>,
    // ObjectIds<VRAM_CUDA>
    >;
    //res.points.resize(lidar_model->size());
    //std::cout << Tbm.size() * model->size() << std::endl;
    ResultT res = lidar->get_data<ResultT>();
  
    // Download results
    Memory<Point, RAM> points = res.points;
  
    const uint32_t points_count = points.size(); // 每帧点数
    sensor_msgs::msg::PointCloud2 msg;
  
    // header
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
  
    // layout: unorganized (height=1)
    msg.height = 1;
    msg.width = points_count;
  
    // fields: x, y, z (FLOAT32 each)
    msg.fields.resize(3);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;
  
    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;
  
    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;
  
    const bool is_bigendian = false;
    msg.is_bigendian = is_bigendian;
    const uint32_t point_step = 12; // 3 * 4 bytes (float32)
    msg.point_step = point_step;
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = true;
  
    // allocate data buffer
    msg.data.resize(msg.row_step * msg.height);
  
    // fill points (random example)
    for (uint32_t i = 0; i < points_count; ++i) {
      float x = points[i].x;
      float y = points[i].y;
      float z = points[i].z;
      // std::cout << points_count << x << y << z << std::endl;
      // copy floats into the byte array at the correct position
      uint32_t byte_index = i * point_step;
      std::memcpy(&msg.data[byte_index + 0], &x, sizeof(float));
      std::memcpy(&msg.data[byte_index + 4], &y, sizeof(float));
      std::memcpy(&msg.data[byte_index + 8], &z, sizeof(float));
    }
  
    pub_pc->publish(msg);
  }
  void publish_camera(OptixCameraPtr camera)
  {
    using ResultT = Bundle<Colors<VRAM_CUDA>>;
    ResultT res = camera->get_data<ResultT>();
    Memory<uint32_t, RAM> colors = res.colors;

    auto msg = sensor_msgs::msg::Image();

    msg.header.stamp = this->now();
    msg.header.frame_id = "camera";

    msg.height = 480;
    msg.width  = 640;
    msg.encoding = "rgb8";     // 非常重要
    msg.is_bigendian = false;
    msg.step = msg.width * 3;  // 每行字节数

    msg.data.resize(msg.step * msg.height);

    // 填充一个简单的测试图（红色）
    for (size_t i = 0; i < colors.size(); i++) {
        msg.data[i*3 + 0] = colors[i] >> 16;
        msg.data[i*3 + 1] = colors[i] >> 8;
        msg.data[i*3 + 2] = colors[i] >> 0;
    }

    pub_img->publish(msg);
  }
  Transform CamT;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_cam;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_front;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_back;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img;
  rclcpp::TimerBase::SharedPtr timer_;
  OptixLidarPtr lidar_front;
  OptixLidarPtr lidar_back;
  // SphereSimulatorOptixPtr lidar_sim;
  // PinholeSimulatorOptixPtr camera_sim;
  // Memory<LiDARModel, RAM> lidar_model;
  // Memory<CameraModel, RAM> camera_model;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OptixRosNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
