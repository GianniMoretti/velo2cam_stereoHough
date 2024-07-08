/*
  velo2cam_stereoHough - Automatic calibration algorithm for extrinsic
  parameters of a stereo camera and a velodyne Copyright (C) 2017-2021 Jorge
  Beltran, Carlos Guindel

  This file is part of velo2cam_stereoHough.

  velo2cam_stereoHough is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  velo2cam_stereoHough is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with velo2cam_stereoHough.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
  stereo_pattern: Find the circle centers in the stereo cloud
*/

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl_msgs/PointIndices.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Empty.h>
#include <message_filters/time_synchronizer.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <iostream>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;
using namespace sensor_msgs;

float baseline = 0.12f;

boost::array<double, 9> PL;
boost::array<double, 9> PR;

int fx = 1;
int fy = 1;
int cx = 1;
int cy = 1;

int frame = 0;

ros::Publisher My_depth_cloud_pub;
ros::Publisher My_comulative_depth_cloud_pub;
std_msgs::Header header_;
ros::Publisher left_camera_ORB;

pcl::PointCloud<pcl::PointXYZ>::Ptr My_comulative_depth_cloud;

std::vector<pcl::PointXYZ> calculate_TD_Centers(std::vector<pcl::PointXY> left_point, std::vector<pcl::PointXY> right_point){
    std::vector<pcl::PointXYZ> TD_centers;
    ROS_INFO("[My Depth]Calculate ORB point...for");
    for(int i = 0; i < left_point.size(); i++){
      double z = ((fx * baseline)/(left_point[i].x - right_point[i].x));
      //Calcola la coordinata X utilizzando la triangolazione stereo
      double x = z * ((left_point[i].x - cx) + (right_point[i].x - cx)) / (2 * fx);
      //Calcola la coordinata Y utilizzando la triangolazione stereo
      //ATTENZIONE: da capire se qui giusto fy
      double y = z * ((left_point[i].y - cy) + (right_point[i].y - cy)) / (2 * fx); //Potrebbe  essere fy
      ROS_INFO("[My Depth]Calculate ORB point...for");
      pcl::PointXYZ p(x, y, z);
      //TD_centers.push_back(p);
    }

    return TD_centers;
}

void callback(const sensor_msgs::ImageConstPtr& image_left,const sensor_msgs::ImageConstPtr& image_right) {

  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try
  {
    cv_ptrLeft = cv_bridge::toCvCopy(image_left, "bgr8");
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrRight;
  try
  {
    cv_ptrRight = cv_bridge::toCvCopy(image_right, "bgr8");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  Mat img1 = cv_ptrLeft->image; 
  Mat img2 = cv_ptrRight->image;

  // if(frame % 120 == 0){
  //   ROS_INFO("Sono entrato");
  //   // Salva l'immagine in locale
  //   std::string output_filename = "image_rect/leftimage" +  std::to_string(frame) + ".jpg";
  //   if (cv::imwrite(output_filename, img1)) {
  //     ROS_INFO("Immagine salvata con successo come %s", output_filename.c_str());
  //   } else {
  //     ROS_INFO("Errore nel salvataggio dell'immagine! %s", output_filename.c_str());
  //   }
  //   // Salva l'immagine in locale
  //   output_filename = "image_rect/rightimage" + std::to_string(frame) + ".jpg";
  //   if (cv::imwrite(output_filename, img2)) {
  //     ROS_INFO("Immagine salvata con successo come %s", output_filename.c_str());
  //   } else {
  //     ROS_INFO("Errore nel salvataggio dell'immagine! %s", output_filename.c_str());
  //   }
  // }
  //Calculate the 3D points
  // // Ridimensiona le immagini se sono troppo grandi
  // if(img1.cols > 800 || img1.rows > 800) {
  //     resize(img1, img1, Size(), 0.5, 0.5);
  // }
  // if (img2.cols > 800 || img2.rows > 800) {
  //     resize(img2, img2, Size(), 0.5, 0.5);
  // }

  // Converti le immagini a scala di grigi
  Mat gray1, gray2; 
  cvtColor(img1, gray1, COLOR_BGR2GRAY); 
  cvtColor(img2, gray2, COLOR_BGR2GRAY); 

  // ROS_INFO("[My Depth]Calculate ORB point...");
  // // Crea il rilevatore orb
  cv::Ptr<Feature2D> orb = cv::ORB::create(2000, 1.2f,8,20);

  // // Rileva le caratteristiche orb
  std::vector<KeyPoint> keypoints1, keypoints2; //però ho paura non sia quello il problema generale comunque
  Mat descriptors1, descriptors2;
  orb->detectAndCompute(gray1, noArray(), keypoints1, descriptors1);
  Mat imgKeypoints1;
  Scalar green(0,255,0);
  drawKeypoints(img1, keypoints1, imgKeypoints1, green);
  sensor_msgs::ImagePtr ros_left_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgKeypoints1).toImageMsg();
  left_camera_ORB.publish(ros_left_image);
  orb->detectAndCompute(gray2, noArray(), keypoints2, descriptors2); 
  //a sto punto li sta prendendo male il matching oppure prendo male le cordinate dei match

  // Crea il matcher
  BFMatcher matcher(NORM_L2, true);
  std::vector<DMatch> matches;
  matcher.match(descriptors1, descriptors2, matches);

  // Ordina le corrispondenze per distanza
  //std::sort(matches.begin(), matches.end());
  std::sort(matches.begin(), matches.end(), [](const DMatch& a, const DMatch& b) {
        // Primo ordina per il primo elemento, poi per il secondo
        if (a.distance >= b.distance) {
            return true;
        }
        else{
            return false;
        }
    });

  // Mantiene solo le migliori corrispondenze
  int numGoodMatches = matches.size() * 0.60;
  matches.erase(matches.begin() + numGoodMatches, matches.end());

  ROS_INFO("[My Depth]Calculate ORB point...2");
  // Estrai i punti di corrispondenza
  std::vector<pcl::PointXY> points1, points2;
  for (size_t i = 0; i < matches.size(); i++) {
      pcl::PointXY point1;
      point1.x = keypoints1[matches[i].queryIdx].pt.x;
      point1.y = keypoints1[matches[i].queryIdx].pt.y;
      points1.push_back(point1);
      pcl::PointXY point2;
      point2.x = keypoints2[matches[i].trainIdx].pt.x;
      point2.y = keypoints2[matches[i].trainIdx].pt.y;
      points2.push_back(point2);
  }

  ROS_INFO("[My Depth]Calculate ORB point...3");
  ROS_INFO("[My Depth] size: %d", points1.size());
  // std::vector<pcl::PointXYZ> tmp = calculate_TD_Centers(points1, points2);
  std::vector<pcl::PointXYZ> TD_centers;

  ROS_INFO("[My Depth]Calculate depths...");
  for(int i = 0; i < points1.size(); i++){
    float z = ((fx * baseline)/(points1[i].x - points2[i].x));
    if(z > 0){
      //Calcola la coordinata X utilizzando la triangolazione stereo
      float x = z * ((points1[i].x - cx) + (points2[i].x - cx)) / (2 * fx);
      //Calcola la coordinata Y utilizzando la triangolazione stereo
      float y = z * ((points1[i].y - cy) + (points2[i].y - cy)) / (2 * fy); //Potrebbe  essere fy, te lo confermo. 
      pcl::PointXYZ* p = new pcl::PointXYZ(x, y, z);
      TD_centers.push_back(*p);
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr My_depth_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  for(int i = 0; i < TD_centers.size(); i++){
    My_depth_cloud->push_back(TD_centers[i]);
    My_comulative_depth_cloud->push_back(TD_centers[i]);
  }

  //Publishing the cloud
  PointCloud2 pub_cloud;
  pcl::toROSMsg(*My_depth_cloud, pub_cloud);
  pub_cloud.header = image_left->header;
  My_depth_cloud_pub.publish(pub_cloud);
  ROS_INFO("[My Depth] Pub final cloud...");

  PointCloud2 pub_com_cloud;
  pcl::toROSMsg(*My_comulative_depth_cloud, pub_com_cloud);
  pub_com_cloud.header = image_left->header;
  My_comulative_depth_cloud_pub.publish(pub_com_cloud);

  frame++;
}

void camInfoCallback(const sensor_msgs::CameraInfoConstPtr &caminfo){
    //Da cambiare metodo di assegnamento
    CamIntrMat = caminfo->K;
    fx = CamIntrMat[0];
    fy = CamIntrMat[4];
    cx = CamIntrMat[2];
    cy = CamIntrMat[5];
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_depth");
    ros::NodeHandle nh;        // GLOBAL
    ros::NodeHandle nh_("~");  // LOCAL

    ros::Subscriber cam_info = nh.subscribe<sensor_msgs::CameraInfo>("zed2i/zed_node/left/camera_info", 1, camInfoCallback);

    message_filters::Subscriber<sensor_msgs::Image> left_image(nh, "image_left", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_image(nh, "image_right", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> AppSync;
    message_filters::Synchronizer<AppSync> sync_(AppSync(100), left_image, right_image);
    sync_.registerCallback(boost::bind(&callback, _1, _2));

    My_depth_cloud_pub = nh_.advertise<PointCloud2>("My_depth_cloud", 1);
    left_camera_ORB = nh_.advertise<sensor_msgs::Image>("left_camera_ORB", 1);
    My_comulative_depth_cloud_pub = nh_.advertise<PointCloud2>("My_comulative_depth_cloud", 1);
    My_comulative_depth_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    ROS_INFO("[My Depth]Starting callback...");
    ros::spin();
    return 0;
}