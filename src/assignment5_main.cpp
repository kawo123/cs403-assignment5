#include <algorithm>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <limits>

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

#define PI 3.14159265 //M_PI

using std::max;
using Eigen::Matrix3f;
using Eigen::Vector3f;
using Eigen::Vector2f;
using geometry_msgs::Point32;
using geometry_msgs::Twist;
using nav_msgs::Odometry;
using std::cout;
using std::vector;

// Robot parameters
const float max_linear_velocity  = 0.5; 
const float max_rotat_velocity = 1.5;
const float max_linear_acceleration  = 0.5; 
const float max_rotat_acceleration = 2;
//const float max_velocity = 0.75; //i'm not sure when we use max_velocity? wasn't in assignment 5
const float robot_radius = 0.18;
const float robot_height = 0.36;
const float delta_time = 0.05; //20 Hz equal to 0.05 seconds
Matrix3f robot_R; 
Vector3f robot_T; 


const float min_angle = -28.0;
const float max_angle = 28.0;
const float increment = 1.0;
const float min_range = 0.8;
const float max_range = 4.0;


const float p_x = 320;
const float p_y = 240;
const float f_x = 588.446;
const float f_y = -564.227;

const float a = 3.008;
const float b = -0.002745;


const float in = std::numeric_limits<float>::infinity();
const Matrix3f R = Matrix3f::Identity();
const Vector3f T(0.13, 0, 0.305);

ros::Publisher part2pub;

// Publisher for velocity command.
ros::Publisher velocity_command_publisher_;

// Last received odometry message.
Odometry last_odometry;

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


  //didn't do this yet.
void CheckPoint(const Vector2f P, const float v, const float w, bool *is_obstacle, float *free_path_length){
    if(w != 0){
      const float R = (fabs(w)>0) ? (v/w): in;
      const Vector2f C(0,R);
      *is_obstacle = fabs((P - C).norm() - R) < robot_radius;
        if (*is_obstacle) {
          const float theta = (w > 0) ? (atan2(P.x(), R - P.y())): atan2(P.x(), P.y() - R);
          //printf("print in middle of method: \n computed theta: %f \nexpected theta: %f\n", theta, 3 * PI/4.0);
          *free_path_length = max(0.0f, (float)(theta*fabs(R) - robot_radius)); //
        }
      else { //not an obstacle
        *free_path_length = max_range;
      }
    }else{ //going straight
      *is_obstacle = fabs(P.y()) < robot_radius;
      if(*is_obstacle){
        *free_path_length = max(0.0f, P.x()-robot_radius);
      }else{
        *free_path_length = max_range;
      }
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
  CheckPoint(P, V.x(), V.y(), &is_obstacle, &free_path_length);

  res.free_path_length = free_path_length;
  res.is_obstacle = is_obstacle;
  return true;
}

bool ObstaclePointCloud(const Matrix3f R, const Vector3f T, const vector<Vector3f> point_cloud, vector<Vector3f> filtered_point_cloud) {
  vector<Vector3f> temp_point_cloud;
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    Vector3f P = R * point_cloud[i] + T; 
    if(P.z() <= robot_height){
      temp_point_cloud.push_back(P); 
    } 
  }

  const float epsilon = 0.02;
  const float pSuccess = 0.95;
  const float pOutliers = 0.35;
  const size_t numIter = (size_t)round(log(1 - pSuccess)/log(1 - pow(1 - pOutliers, 3)));
  ROS_INFO("numIter: %lu", numIter);

  srand(time(NULL));

  for (size_t i = 0; i < numIter; ++i){
    ROS_INFO("iterating: %lu", i);
    Vector3f P1 = temp_point_cloud[rand() % temp_point_cloud.size()];
    Vector3f P2 = temp_point_cloud[rand() % temp_point_cloud.size()];
    Vector3f P3 = temp_point_cloud[rand() % temp_point_cloud.size()];

    Vector3f n = (P2 - P1).cross(P3 - P1);
    n = n/n.norm();
    Vector3f P0 = P1;

    filtered_point_cloud.clear();
    for (size_t i = 0; i < temp_point_cloud.size(); ++i) {
      if (fabs(n.dot(temp_point_cloud[i] - P0)) > epsilon) {
        filtered_point_cloud.push_back(temp_point_cloud[i]);
      }
    }

    ROS_INFO("percent inliers: %f", 1 - (((float)filtered_point_cloud.size())/((float)point_cloud.size())));
    if (pOutliers >= (((float)filtered_point_cloud.size())/((float)point_cloud.size()))){
      ROS_INFO("successfuly found ground plain");
      break;
    }
  }
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

  ObstaclePointCloud(R, T, point_cloud, filtered_point_cloud);
  

  res.P_prime.resize(filtered_point_cloud.size());
  for (size_t i = 0; i < filtered_point_cloud.size(); ++i) {
    res.P_prime[i] = ConvertVectorToPoint(filtered_point_cloud[i]);
  }
  return true;
}

bool PointCloudToLaserScan(const vector<Vector3f> point_cloud, vector<float> ranges){
  const int size = (int) max_angle - min_angle + 1;
  ranges.resize(size); 

  for (size_t i = 0; i < size; ++i) {
    ranges[i] = 0; 
  }

  for (size_t i = 0; i < point_cloud.size(); ++i) {
    const float angle = atan(point_cloud[i].y() / point_cloud[i].x()) * 180 / PI;
    const float rounded_angle = round(angle); 
    if(rounded_angle >= min_angle && rounded_angle <= max_angle){
      const int index = (int)(rounded_angle + 28.0); 
      const float distance = FindVectorMaginitude(point_cloud[i].x(), point_cloud[i].y());//point_cloud[i].norm()
      if(distance < ranges[index]){
        ranges[index] = distance; 
      }
    }
  }


 /* vector<Vector3f> minimum_points_in_point_cloud;
  //for each increment in ranges in laser scan
  //look at the minimum point in the range its current looking at

  
  for(size_t i = 0; i<ranges.size(); ++i){
    Vector3f minimum_point;
    for(size_t j = 0 ; j <filtered_point_cloud.size(); ++j){

      const float angle = atan(filtered_point_cloud[i].y() / filtered_point_cloud[i].x()) * 180 / PI; //should be in radians. depemding if increment is in radians

      const float directionalangle = min_angle+increment* static_cast<float>(i);
      if(angle<=directionalangle+increment/2 && angle>directionalangle-increment/2){ //atan is within the increment
        float minimum_point_distance = FindVectorMaginitude(minimum_point.x(), minimum_point.y()); 
        const float distance = FindVectorMaginitude(filtered_point_cloud[i].x(), filtered_point_cloud[i].y()); 
          if(distance < minimum_point_distance){
            const Vector3f P = filtered_point_cloud[i];//(filtered_point_cloud[i].x(), filtered_point_cloud[i].y());
            minimum_point = P;
            minimum_points_in_point_cloud.push_back(minimum_point);

        }
    }
    }
  }*/
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

  PointCloudToLaserScan(point_cloud, ranges);

  res.ranges = ranges;
  return true;
}



void GetCommandVel(const Matrix3f R, const Vector3f T, const sensor_msgs::Image Image,const float v0,const float w0, float *linear_velocity, float *rot_velocity){
 // const Vector2f V(v0.x, w0.z);
  float v = v0;
  float w = w0;
  vector<Vector3f> point_cloud;
  int count = 10;
  for (unsigned int y = 0; y < Image.height; ++y) {
    for (unsigned int x = 0; x < Image.width; ++x) {
      // Add code here to only process only every nth pixel
      if(count <= 0){
        uint16_t byte0 = Image.data[2 * (x + y * Image.width) + 0];
        uint16_t byte1 = Image.data[2 * (x + y * Image.width) + 1];
        if (!Image.is_bigendian) {
          std::swap(byte0, byte1);
        }
        // Combine the two bytes to form a 16 bit value, and disregard the
        // most significant 4 bits to extract the lowest 12 bits.
        const uint16_t raw_depth = ((byte0 << 8) | byte1) & 0x7FF;
        // Reconstruct 3D point from x, y, raw_depth using the camera intrinsics and add it to your point cloud.
       float depth = 1/ (a + (b*raw_depth));

        Vector3f point(depth * ((x - p_x)/f_x),  depth * ((y - p_y)/f_y), depth);
        point_cloud.push_back(point);
        count = 10;
    } else {
      count--;
    }
   } 
  }


  vector<Vector3f> filtered_point_cloud;

  ObstaclePointCloud(R, T, point_cloud, filtered_point_cloud);

  // Use your code from part 3 to convert the point cloud to a laser scan
    vector<float> ranges; //laser scan

  const int size = (int) max_angle - min_angle + 1;
  ranges.resize(size); 

  for (size_t i = 0; i < ranges.size(); ++i) {
    ranges[i] = 0; //initializes
  }

  PointCloudToLaserScan(filtered_point_cloud, ranges);




  //dynamic window of minimum and maximum velocities given acceleration constraints

//changes
  const float Vmin = v - max_linear_acceleration*delta_time; 
  const float Vmax = v + max_linear_acceleration*delta_time;
  const float Wmin = w - max_rotat_acceleration*delta_time;
  const float Wmax = w + max_rotat_acceleration*delta_time;


  //size of acceleration dynamic window
  const float Vdifference = Vmax- Vmin;
  const float Wdifference = Wmax - Wmin;

  //50 values in each
  const float vincrement = Vdifference/50;
  const float wincrement = Wdifference/50;

  const int Vsize = 51;
  const int WSize = 51;

  float min_free_path = 5; //in meters?

  float dynamicwindow[Vsize][WSize];
  float G = 0;

  const float alpha = 1;
  const float beta = 1;
  const float tao = 1;
  const float sigma = 1;

  for(size_t currentv = 0; currentv<Vsize; currentv = currentv+vincrement){
    for(size_t currentw = 0; currentw<WSize; currentw = currentw+wincrement){
    //admissible velocities for dynamic window
      if(abs(currentv)<max_linear_velocity && abs(currentw)<max_rotat_velocity){
        //best velocity
        const float currentlinearvelocity = Vmin + vincrement* static_cast<float>(currentv);
        const float currentrotvelocity = Wmin + wincrement* static_cast<float>(currentw);
        float free_path_length = 0;
        bool is_obstacle = false;

        float theta = min_angle;
        for(size_t i = 0; i<ranges.size(); ++i){
          const float radtheta = theta * PI/180;
          const Vector2f P(sin(radtheta)*ranges[i], cos(radtheta)*ranges[i]);
          CheckPoint(P, currentlinearvelocity, currentrotvelocity, &is_obstacle, &free_path_length);
          if(is_obstacle && free_path_length < min_free_path){
            min_free_path = free_path_length;
            /*v = currentlinearvelocity;
            w = currentrotvelocity;*/

          }
            theta+=increment;
          
      } 
        if(free_path_length >= min_range && currentlinearvelocity < sqrt(2 * max_linear_acceleration * free_path_length)){
          float current_G = sigma * (alpha * (max_rotat_velocity-abs(currentrotvelocity)) + 
          					beta * free_path_length + tao * currentlinearvelocity);
          if(current_G > G){
            G = current_G;
            v = currentlinearvelocity;
            w = currentrotvelocity;
          }
        }


      }
    }
  }

*linear_velocity = v;
*rot_velocity = w;


/*

  //compute free path and collect all the
    //free path lengths to the obstacles
  vector<float> values(minimum_points_in_point_cloud.size(), 0.0);


  for(size_t i = 0; i<minimum_points_in_point_cloud.size(); ++i){
    bool current_obstacle=false;
    float current_path_length = max_range;
    CheckPoint(minimum_points_in_point_cloud[i], v, w, &current_obstacle, &current_path_length);
    values[i] = current_path_length;
  }*/


  // Implement dynamic windowing approach to find the best velocity command for next time step

  //once you get all the possible free path lengths, you'll have to find the max of the free path lengths
  //once you do, get the velocity of it
 /* Vector2f V_tilde = V;
  for(size_t i =0; i<values.size(); ++i){
    if(values[i] >= max_free_path){
      max_free_path = values[i];
      //get the velocity of tht direction. should store the direction though....
      const float angle = min_angle + increment * static_cast<float>(i);
      const Vector2f dir(cos(angle), sin(angle));
      V_tilde = FindVectorMaginitude(dir) * values[i];
      Cv = V_tilde; //Cv is supposed to be a geometry_msg
      Cw = angle;
      break;
        //this doesnt have to do with amax or vmax at all? is tht only to help see if the V_tilde is admissible
        //how to dynamic window? dynamic window is restrained for your amax and vmax
    } 
  }
*/
}


bool GetCommandVelService(
    compsci403_assignment5::GetCommandVelSrv::Request& req,
    compsci403_assignment5::GetCommandVelSrv::Response& res) {

  float linear_velocity = 0;
  float rot_velocity = 0;
  //geometry_msgs::Point32 Cv = req.v0;
  //geometry_msgs::Point32 Cw = req.w0;

  GetCommandVel(R, T, req.Image,req.v0.x, req.w0.z, &linear_velocity, &rot_velocity);


  res.Cv.x = linear_velocity;
  res.Cw.z = rot_velocity;

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
  float linear_velocity = 0;
  float rot_velocity = 0;
  GetCommandVel(robot_R, robot_T, depth_image, v0, w0, &linear_velocity, &rot_velocity);

  command_vel.linear.x = linear_velocity; // replace with your calculated linear velocity c_v
  command_vel.angular.z = rot_velocity; // replace with your angular calculated velocity c_w
  velocity_command_publisher_.publish(command_vel);
}

void TestCheckPoint(){
  const float v= 1;
  const float w= 2;
  const float theta_expected = PI/4.0;
  const float free_path_expected = PI/8.0;
  const Vector2f p(0.25, 0.25);
  float free_path_length = 0;
  bool is_obstacle = false;
  CheckPoint(p,v,w,&is_obstacle,&free_path_length);
  //reproduce code here. if u get radius to point p obstacle, you can see if it's obstacle
  //can see if it's an obstacle
    printf("free_path_length: %f \n is_obstacle: %d", free_path_length, is_obstacle);
        printf("\nEXPECTED free_path_length: %f \n EXPECTED theta expected: %f\n", free_path_expected, theta_expected);

}



void TestCheckPoint2(){
  const float v= 1;
  const float w= 2;
  const float theta_expected = 3 * PI/4.0;
  const float free_path_expected = 3 * PI/8.0;
  const Vector2f p(0.25, 0.75);
  float free_path_length = 0;
  bool is_obstacle = false;
  CheckPoint(p,v,w,&is_obstacle,&free_path_length);
  //reproduce code here. if u get radius to point p obstacle, you can see if it's obstacle
  //can see if it's an obstacle
    printf("free_path_length: %f \n is_obstacle: %d", free_path_length, is_obstacle);
        printf("\nEXPECTED free_path_length: %f \n EXPECTED theta expected: %f\n", free_path_expected, theta_expected);

}

void part2tester(const sensor_msgs::PointCloud& point_cloud_msg){
  sensor_msgs::PointCloud filtered_point_cloud_msg;
  filtered_point_cloud_msg.header = point_cloud_msg.header;
  // Create a Vector3f point cloud, of the same size as the input point cloud.
  vector<Vector3f> point_cloud(point_cloud_msg.points.size());

  // Copy over the input point cloud.
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    point_cloud[i] = ConvertPointToVector(point_cloud_msg.points[i]);
  }

  vector<Vector3f> filtered_point_cloud;
  ObstaclePointCloud(R, T, point_cloud, filtered_point_cloud);

  filtered_point_cloud_msg.points.resize(filtered_point_cloud.size());
  for (size_t i = 0; i < filtered_point_cloud.size(); ++i) {
    filtered_point_cloud_msg.points[i] = ConvertVectorToPoint(filtered_point_cloud[i]);
  }
  ROS_INFO("part2tester called");
  part2pub.publish(filtered_point_cloud_msg);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "compsci403_assignment5");
  ros::NodeHandle n;

  if(false){
    //TestCheckPoint1();
    TestCheckPoint2();
    return true;
  }

  // Write client node to get R and T from GetTransformationSrv
  ros::ServiceClient client = n.serviceClient<compsci403_assignment5::GetTransformationSrv>
    ("/COMPSCI403/GetTransformation");
  compsci403_assignment5::GetTransformationSrv srv; 
  if(client.call(srv)){
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        robot_R(row, col) = srv.response.R[col * 3 + row];
      }
    }
    robot_T.x() = srv.response.T.x;
    robot_T.y() = srv.response.T.y;
    robot_T.z() = srv.response.T.z; 

  }else{
    ROS_ERROR("Failed to call service GetTransformationSrv"); 
    return 1; 
  }

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

  part2pub = 
      n.advertise<sensor_msgs::PointCloud>("testing_part_2", 1);

  ros::Subscriber depth_image_subscriber =
      n.subscribe("/Cobot/Kinect/Depth", 1, DepthImageCallback);
  ros::Subscriber odometry_subscriber =
      n.subscribe("/odom", 1, OdometryCallback);

  ros::Subscriber part2sub =
      n.subscribe("/Cobot/Kinect/FilteredPointCloud", 1, part2tester);

  ros::spin();

  return 0;
}