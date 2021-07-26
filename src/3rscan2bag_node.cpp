
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
// #include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <cv_bridge/cv_bridge.h>

#include <SemanticTypes.h>

#include <rio_lib/rio.h>

using namespace std;
string root_dir,scan_id,scan_dir, output_name;
string rgb_topic, depth_topic, pose_topic, bbox_topic, path_topic, rawdepth_topic;
string frame_name;
bool output_raw_data;
int N_MAX;
double t_gap;

// obj
rosbag::Bag bag; 
vector<geometry_msgs::PoseStamped> pose_db;

//RIO object
RIO::RIO *rio_database;
RIO::Scan scan_data;

bool write_rgb_img(const int &frame_id)
{
    sensor_msgs::Image img_msg;
    cv_bridge::CvImage img_cv_bridge;
    stringstream rgb_dir;
    double timestamp_sec = (frame_id+1) * t_gap; 

    rgb_dir<<scan_dir<<"sequence/frame-"<< std::setfill('0') << std::setw(6)<< frame_id
      <<".color.jpg";
    cv::Mat rgb_img = cv::imread(rgb_dir.str(),cv::IMREAD_COLOR);

    if(rgb_img.empty()){
      ROS_WARN("RGB not found at %s",rgb_dir.str().c_str());
      return false;
    }

    if(!output_raw_data){
      cv::rotate(rgb_img,rgb_img,cv::ROTATE_90_CLOCKWISE);
    }

    img_cv_bridge.header.seq = frame_id;
    img_cv_bridge.header.stamp.fromSec(timestamp_sec);
    img_cv_bridge.image = rgb_img;
    img_cv_bridge.encoding = "bgr8";
    img_cv_bridge.header.frame_id = frame_name;

    img_cv_bridge.toImageMsg(img_msg);
    // cv::imshow("debug",rgb_img);
    // cv::waitKey(100);
    bag.write(rgb_topic,img_msg.header.stamp,img_msg);
    cout<<rgb_dir.str()<<"\n";
    
    return true;
}

bool write_raw_depth_img(const int &frame_id)
{
    sensor_msgs::Image img_msg;
    cv_bridge::CvImage img_cv_bridge;
    stringstream depth_dir;
    double timestamp_sec = (frame_id+1) * t_gap; 

    depth_dir<<scan_dir<<"sequence/frame-"<< std::setfill('0') << std::setw(6)<< frame_id
      <<".depth.pgm";
    cv::Mat depth_img = cv::imread(depth_dir.str(),-1);

    if(!output_raw_data){
      cv::rotate(depth_img,depth_img,cv::ROTATE_90_COUNTERCLOCKWISE);
    }

    if(depth_img.empty()){
      ROS_WARN("Depth not found");
      return false;
    }

    img_cv_bridge.header.seq = frame_id;
    img_cv_bridge.header.stamp.fromSec(timestamp_sec);
    img_cv_bridge.image = depth_img;
    img_cv_bridge.encoding = sensor_msgs::image_encodings::MONO16;
    img_cv_bridge.header.frame_id = frame_name;
    

    img_cv_bridge.toImageMsg(img_msg);
    bag.write(rawdepth_topic,img_msg.header.stamp,img_msg);
    
    return true;
}

bool write_depth_img(const int &frame_id)
{
    sensor_msgs::Image img_msg;
    cv_bridge::CvImage img_cv_bridge;
    stringstream depth_dir;
    double timestamp_sec = (frame_id+1) * t_gap; 

    depth_dir<<scan_dir<<"sequence/frame-"<< std::setfill('0') << std::setw(6)<< frame_id
      <<".rendered.depth.png";
    cv::Mat depth_img = cv::imread(depth_dir.str(),cv::IMREAD_GRAYSCALE);

    if(output_raw_data){
      cv::rotate(depth_img,depth_img,cv::ROTATE_90_COUNTERCLOCKWISE);
    }

    if(depth_img.empty()){
      ROS_WARN("Depth not found");
      return false;
    }

    // cv::rotate(depth_img,depth_img,cv::ROTATE_90_CLOCKWISE);
    img_cv_bridge.header.seq = frame_id;
    img_cv_bridge.header.stamp.fromSec(timestamp_sec);
    img_cv_bridge.image = depth_img;
    img_cv_bridge.encoding = sensor_msgs::image_encodings::MONO8;
    img_cv_bridge.header.frame_id = frame_name;

    img_cv_bridge.toImageMsg(img_msg);
    bag.write(depth_topic,img_msg.header.stamp,img_msg);
    // cout<<depth_dir.str()<<"\n";
    
    return true;
}

bool write_pose_msg(const int &frame_id)
{
  stringstream pose_dir;
  string line_str;
  Eigen::Quaterniond q_;
  Eigen::Matrix<double,4,4> T_;
  string T_str[4];
  geometry_msgs::PoseStamped pose_msg;
  nav_msgs::Path path_msg;

  Eigen::Matrix<double,3,3> rot;
  if(output_raw_data){
    rot.setIdentity();
  }
  else {
    rot = Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(-M_PI_2,Eigen::Vector3d::UnitY());
  }


  double timestamp_sec = (frame_id+1) * t_gap; 
  pose_dir<<scan_dir<<"sequence/frame-"<< std::setfill('0') << std::setw(6)<< frame_id
    <<".pose.txt";
  ifstream pose_file(pose_dir.str());
  if(!pose_file.is_open()){
    ROS_WARN("Pose not found");
    return false;
  }

  int j=0;
  T_.setIdentity();
  while(getline(pose_file,line_str)){
    istringstream buffer(line_str);
    buffer>>T_str[0]>>T_str[1]>>T_str[2]>>T_str[3];
    T_.row(j)<<atof(T_str[0].c_str()),atof(T_str[1].c_str()),atof(T_str[2].c_str()),
      atof(T_str[3].c_str());
    j++;
  }

  T_.topLeftCorner(3,3) = T_.topLeftCorner(3,3) * rot;
  // std::cout<<"Pose: \n"<<T_<<"\n";

  // pose msg
  pose_msg.header.seq = frame_id;
  pose_msg.header.frame_id = frame_name;
  pose_msg.header.stamp.fromSec(timestamp_sec);
  q_ = T_.block<3,3>(0,0);
  pose_msg.pose.position.x = T_(0,3);
  pose_msg.pose.position.y = T_(1,3);
  pose_msg.pose.position.z = T_(2,3);
  pose_msg.pose.orientation.w = q_.w();
  pose_msg.pose.orientation.x = q_.x();
  pose_msg.pose.orientation.y = q_.y();
  pose_msg.pose.orientation.z = q_.z();
  bag.write(pose_topic,pose_msg.header.stamp,pose_msg);

  pose_db.emplace_back(pose_msg);

  // path msg
  path_msg.header = pose_msg.header;
  for (auto pose_iter:pose_db){
    path_msg.poses.emplace_back(pose_iter);
  }
  bag.write(path_topic,path_msg.header.stamp,path_msg);

  return true;

}

bool saveSemanticType(const int &query_type){
  if(valid_semantic_types.find(query_type)!=valid_semantic_types.end()){
    // std::cout<<"Available! \n";
    return true;
  }
  else{
    // std::cout<<"Wrong! \n";
    return false;
  }
}

bool write_bbox_msg(const int &frame_id)
{
  // ifstream bbox_file;
  stringstream file_dir;
  string line_str;
  geometry_msgs::PoseArray bboxs_array;

  file_dir<<scan_dir<<"sequence/frame-"<< std::setfill('0') << std::setw(6)<< frame_id
    <<".bb.txt";
  ifstream bbox_file(file_dir.str());
  
  if(bbox_file.is_open()){
    double timestamp_sec = double(frame_id+1) * t_gap; 
    bboxs_array.header.seq = frame_id;
    bboxs_array.header.stamp.fromSec(timestamp_sec);

    while (getline(bbox_file,line_str))
    {
      istringstream buf(line_str);
      string id,u0,v0,u1,v1;
      geometry_msgs::Pose bbox_msg;
      
      buf >> id>>u0>>v0>>u1>>v1;
      int semantic_object_id = atof(id.c_str());
      int query_type = scan_data.instance2global.at(semantic_object_id);

      if(!saveSemanticType(query_type)){
        continue;
      }
      // std::cout<<"Save obj id:"<<semantic_object_id<<","
      //   <<"global type id:"<<query_type<<"\n";

      bbox_msg.position.x = atof(u0.c_str());
      bbox_msg.position.y = atof(v0.c_str());
      bbox_msg.position.z = atof(id.c_str());
      bbox_msg.orientation.x = atof(u1.c_str());
      bbox_msg.orientation.y = atof(v1.c_str());
      bbox_msg.orientation.z = query_type;

      bboxs_array.poses.emplace_back(bbox_msg);
      // std::cout<<buf.str()<<"\n";
    }
    bag.write(bbox_topic,bboxs_array.header.stamp,bboxs_array);

    return true;
  }
  else {
    ROS_WARN("BBox not found");

    return false;
  }



}

void process_frames()
{
  for(int i=0;i<N_MAX;i++)
  {
    if(!write_rgb_img(i) || !write_depth_img(i)|| !write_pose_msg(i)
      || !write_bbox_msg(i) || !write_raw_depth_img(i)){
      ROS_WARN("Frame %d read empty!",i);
      return;
    }
  }
}


int main(int argc, char** argv) 
{
  ros::init(argc, argv, "3rscan2bag_node");
  ros::NodeHandle n("~");

  // Paramters
  // t_gap = 0.1;  //10 Hz
  rgb_topic  = "rgb";
  depth_topic = "depth";
  pose_topic = "camera_pose";
  bbox_topic = "bbox";
  path_topic = "path";
  rawdepth_topic = "raw_depth";
  n.getParam("root_dir",root_dir);
  n.getParam("scan_id",scan_id);
  n.getParam("max_image_num",N_MAX);
  n.getParam("frame_name",frame_name);
  n.getParam("t_gap",t_gap);
  // n.getParam("output_name",output_name);
  n.getParam("output_raw_data",output_raw_data);
  if(output_raw_data){
    output_name = "rawdata";
  }
  else output_name = "rotatedata";

  stringstream scan_dir_stream;
  scan_dir_stream<<root_dir<<"/"<<scan_id<<"/";
  scan_dir = scan_dir_stream.str();
  ROS_WARN("Process sequence data:%s",scan_dir.c_str());
  // std::cout<<"Scan dir: "<< scan_dir<<"\n";
  std::cout<<"frame name:"<<frame_name<<"\n";

  //
  const RIO::RIOConfig config(root_dir);
  rio_database = new RIO::RIO(config);
  if(!rio_database->getScanFromID(scan_id,scan_data)){
    return 0;
  }

  //
  stringstream output_bag_dir;
  output_bag_dir <<scan_dir<<output_name<<".bag";
  bag.open(output_bag_dir.str(),rosbag::bagmode::Write);

  process_frames();

  ROS_WARN("Rosbag created at: %s!",output_bag_dir.str().c_str());


  ros::spin();

  return 0;
}