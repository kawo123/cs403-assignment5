#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "compsci403_assignment5/CheckPointSrv.h"
#include "compsci403_assignment5/ObstaclePointCloudSrv.h"
#include "compsci403_assignment5/GetCommandVelSrv.h"
#include "compsci403_assignment5/GetTransformationSrv.h"
#include "compsci403_assignment5/PointCloudToLaserScanSrv.h"

#define PI 3.14159265

using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::Vector2f;
using geometry_msgs::Point32;
using geometry_msgs::Twist;
using nav_msgs::Odometry;
using std::cout;
using std::vector;


// Publisher for velocity command.
ros::Publisher velocity_command_publisher_;

// Last received odometry message.
Odometry last_odometry;

// Robotic parameters
const float max_linear_velocity  = 0.5; 
const float max_rotat_velocity = 1.5;
const float max_linear_acceleration  = 0.5; 
const float max_rotat_acceleration = 2;
const float max_velocity = 0.75; 
const float robot_radius = 0.18;
const float robot_height = 0.36;

// Helper function to convert ROS Point32 to Eigen Vectors.
Vector3f ConvertPointToVector(const Point32& point) {
  return Vector3f(point.x, point.y, point.z);
}

// Helper function to convert Eigen Vectors to ROS Point32.
Point32 ConvertVectorToPoint(const Vector3f& vector) {
  Point32 point;
  point.x = vector.x();
  point.y = vector.y();
  point.z = vector.z();
  return point;
}

// Helper function to find the magnitude of Vector2f
float FindVectorMaginitude(const Vector2f V){
  return (sqrt(pow(V.x(), 2)+pow(V.y(), 2))); 
}

// Helper function to find the magnitude of x and y
float FindVectorMaginitude(const float x, const float y){
  return (sqrt(pow(x, 2)+pow(y, 2))); 
}

void GetFreePath(const sensor_msgs::LaserScan& ranges,
                 const Vector2f& V,
                 bool* is_obstacle,
                 float* free_path_length) {
//given linear velocity, angular velocity 
  //which is turned into V and point

  const float min_angle = -28.0;
  const float max_angle = 28.0;
  const float increment = 1.0;
  const float min_range = 0.8;
  const float max_range = 4.0;

  *is_obstacle = false;
  *free_path_length = max_range + robot_radius;
  for(size_t i =0; i<ranges.size(); ++i){
    //ignore invalid ranges
    if(ranges[i] < min_range || ranges[i]>max_range){
      continue;
    }
    //compute angle
    const float angle = min_angle + increment * static_cast<float>(i);
    //compute laser point
    const Vector2f P = scan.ranges[i] * Vector2f(cos(angle), sin(angle));
    // ignore points that are "behind" the robot.
    if (P.dot(V) <= 0.0) continue;
    //use a method to calculate the free path
    // of current point and check if obstacles
    float current_path_length = scan.range_max + kRobotRadius;
    bool current_obstacle = false;
    *free_path_length = min(current_path_length, *free_path_length);
    //why minimum of the free paths??
    *is_obstacle = *is_obstacle || current_obstacle;
  }

}

bool CheckPointService(
    compsci403_assignment5::CheckPointSrv::Request& req,
    compsci403_assignment5::CheckPointSrv::Response& res) {
  // Observed point.
  const Vector2f P(req.P.x, req.P.y);
  // Desired velocity vector.
  const Vector2f V(req.v.x, req.w.z);
  bool is_obstacle = false;
  float free_path_length = 0.0;

  // Write code to compute is_obstacle and free_path_length.

  float rotation_radius = V.x() / V.y(); 
  const Vector2f C(0, rotation_radius);
  const float distance = FindVectorMaginitude(C - P);
  if(distance < (robot_radius + rotation_radius)
    || distance > (robot_radius - rotation_radius)){
    is_obstacle = true; 
    // calculate free path
  }

  //////////////////////////////////////////////////////////

  res.free_path_length = free_path_length;
  res.is_obstacle = is_obstacle;
  return true;
}

bool ObstaclePointCloudService(
    compsci403_assignment5::ObstaclePointCloudSrv::Request& req,
    compsci403_assignment5::ObstaclePointCloudSrv::Response& res) {
  Matrix3f R;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      R(row, col) = req.R[col * 3 + row];
    }
  }
  const Vector3f T(req.T.x, req.T.y, req.T.z);

  // Copy over all the points.
  vector<Vector3f> point_cloud(req.P.size());
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    point_cloud[i] = ConvertPointToVector(req.P[i]);
  }

  vector<Vector3f> filtered_point_cloud;
  // Write code here to transform the input point cloud from the Kinect reference frame to the
  // robot's reference frame. Then filter out the points corresponding to ground
  // or heights larger than the height of the robot
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    const Vector3f robot_frame_point = R * point_cloud[i] + T; 
    if(!(robot_frame_point.z() >=  0
        || robot_frame_point.z() <= robot_height)){ // 
      filtered_point_cloud.push_back(robot_frame_point); 
    } 
  }

  res.P_prime.resize(filtered_point_cloud.size());
  for (size_t i = 0; i < filtered_point_cloud.size(); ++i) {
    res.P_prime[i] = ConvertVectorToPoint(filtered_point_cloud[i]);
  }
  return true;
}

bool PointCloudToLaserScanService(
    compsci403_assignment5::PointCloudToLaserScanSrv::Request& req,
    compsci403_assignment5::PointCloudToLaserScanSrv::Response& res) {

  // Copy over all the points.
  vector<Vector3f> point_cloud(req.P.size());
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    point_cloud[i] = ConvertPointToVector(req.P[i]);
  }

  vector<float> ranges;
  // Process the point cloud here and convert it to a laser scan

  const float min_angle = -28.0;
  const float max_angle = 28.0;
  const float increment = 1.0;
  const float min_range = 0.8;
  const float max_range = 4.0; 
  const int size = (int) max_angle - min_angle + 1;
  ranges.resize(size); 
  // vector<float> closest_angle(size); 

  for (size_t i = 0; i < ranges.size(); ++i) {
    ranges[i] = 0; 
    // closest_angle[i] = INT_MIN; 
  }

  for (size_t i = 0; i < point_cloud.size(); ++i) {
    const float angle = atan(point_cloud[i].y() / point_cloud[i].x()) * 180 / PI;
    const float rounded_angle = round(angle); 
    if(rounded_angle >= min_angle && rounded_angle <= max_angle){
      const int index = (int)(rounded_angle + 28.0);
      const float distance = FindVectorMaginitude(point_cloud[i].x(), point_cloud[i].y()); 
      if(distance < ranges[index]){
        ranges[index] = distance; 
      }
    }

  }
  /////////////////////////////////////////////////////////////

  res.ranges = ranges;
  return true;
}

bool GetCommandVelService(
    compsci403_assignment5::GetCommandVelSrv::Request& req,
    compsci403_assignment5::GetCommandVelSrv::Response& res) {

  vector<Vector3f> point_cloud;
  // The input v0 and w0 are each vectors. The x component of v0 is the linear 
  // velocity towards forward direction and the z component of w0 is the
  // rotational velocity around the z axis, i.e. around the center of rotation
  // of the robot and in counter-clockwise direction
  const Vector2f V(req.v0.x, req.w0.z);


  for (unsigned int y = 0; y < req.Image.height; ++y) {
    for (unsigned int x = 0; x < req.Image.width; ++x) {
      // Add code here to only process only every nth pixel

      uint16_t byte0 = req.Image.data[2 * (x + y * req.Image.width) + 0];
      uint16_t byte1 = req.Image.data[2 * (x + y * req.Image.width) + 1];
      if (!req.Image.is_bigendian) {
        std::swap(byte0, byte1);
      }
      // Combine the two bytes to form a 16 bit value, and disregard the
      // most significant 4 bits to extract the lowest 12 bits.
      const uint16_t raw_depth = ((byte0 << 8) | byte1) & 0x7FF;
      // Reconstruct 3D point from x, y, raw_depth using the camera intrinsics and add it to your point cloud.
      Vector3f point;

      point_cloud.push_back(point);
    }
  }

  // Use your code from part 3 to convert the point cloud to a laser scan
    vector<float> ranges; //laser scan

  const float min_angle = -28.0;
  const float max_angle = 28.0;
  const float increment = 1.0;
  const float min_range = 0.8;
  const float max_range = 4.0; 
  const int size = (int) max_angle - min_angle + 1;
  ranges.resize(size); 



  for (size_t i = 0; i < ranges.size(); ++i) {
    ranges[i] = 0; //initializes
  }

  for (size_t i = 0; i < point_cloud.size(); ++i) {
    const float angle = atan(point_cloud[i].y() / point_cloud[i].x()) * 180 / PI;
  //why times 180 / PI?

    const float rounded_angle = round(angle); 

    //why rounded angle?
    if(rounded_angle >= min_angle && rounded_angle <= max_angle){
      const int index = (int)(rounded_angle + 28.0);
      //why not using increment?
      const float distance = FindVectorMaginitude(point_cloud[i].x(), point_cloud[i].y()); 
      if(distance < ranges[index]){
        ranges[index] = distance; 
      }
    }
  }
  // Implement dynamic windowing approach to find the best velocity command for next time step

  // Return the best velocity command
  //given V0 and depth image which is turned into
  // point cloud. point cloud turns into laser scan
  //laser scan turns into best direction?
  //best direction turns into best velocity.
  


  /*for(size_t i = 0; i<ranges.size(); ++i){
    const float angle = min_angle + increment * static_cast<float>(i);
    const Vector2f dir(cos(angle), sin(angle));
  }

  //ignore directions that lead backwards from V.
  if(V.dot(dir) <= 0.0) continue; //????

  //compute free path so you can maximize it
  float current_path_length = max_range + robot_radius;
  bool current_obstacle = false;
  GetFreePath(ranges, V, current_obstacle, current_path_length)



  float best_velocity = 0;*/

  // Cv is of type Point32 and its x component is the linear velocity towards forward direction
  // you do not need to fill its other components
  res.Cv.x = 0;
  // Cw is of type Point32 and its z component is the rotational velocity around z axis
  // you do not need to fill its other components
  res.Cw.z = 0;

  return true;
}

void OdometryCallback(const nav_msgs::Odometry& odometry) {
  last_odometry = odometry;
}

void DepthImageCallback(const sensor_msgs::Image& depth_image) {
  Twist command_vel;
  // Current velocity
  const float v0 = last_odometry.twist.twist.linear.x;
  const float w0 = last_odometry.twist.twist.angular.z;

  // Use your code from all other parts to process the depth image, 
  // find the best velocity command and publish the velocity command


  command_vel.linear.x = 0; // replace with your calculated linear velocity c_v
  command_vel.angular.z = 0; // replace with your angular calculated velocity c_w
  velocity_command_publisher_.publish(command_vel);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "compsci403_assignment5");
  ros::NodeHandle n;

  ros::ServiceServer service1 = n.advertiseService(
      "/COMPSCI403/CheckPoint", CheckPointService);
  ros::ServiceServer service2 = n.advertiseService(
      "/COMPSCI403/ObstaclePointCloud", ObstaclePointCloudService);
  ros::ServiceServer service3 = n.advertiseService(
      "/COMPSCI403/PointCloudToLaserScan", PointCloudToLaserScanService);
  ros::ServiceServer service4 = n.advertiseService(
      "/COMPSCI403/GetCommandVel", GetCommandVelService);

  velocity_command_publisher_ =
      n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

  ros::Subscriber depth_image_subscriber =
      n.subscribe("/Cobot/Kinect/Depth", 1, DepthImageCallback);
  ros::Subscriber odometry_subscriber =
      n.subscribe("/odom", 1, OdometryCallback);

  ros::spin();

  return 0;
}










