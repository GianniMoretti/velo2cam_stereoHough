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

double baseline = 0.12;

boost::array<double, 9> CamIntrMat;

int fx;
int fy;
int cx;
int cy;

ros::Publisher My_depth_cloud_pub;
std_msgs::Header header_;

std::vector<pcl::PointXYZ> calculate_TD_Centers(std::vector<pcl::PointXY> left_point, std::vector<pcl::PointXY> right_point){
    std::vector<pcl::PointXYZ> TD_centers;
    for(int i = 0; i < 4; i++){
        double z = ((fx * baseline)/(left_point[i].x - right_point[i].x));
        //Calcola la coordinata X utilizzando la triangolazione stereo
        double x = z * ((left_point[i].x - cx) + (right_point[i].x - cx)) / (2 * fx);
        //Calcola la coordinata Y utilizzando la triangolazione stereo
        //ATTENZIONE: da capire se qui giusto fy
        double y = z * ((left_point[i].y - cy) + (right_point[i].y - cy)) / (2 * fx); //Potrebbe  essere fy

        pcl::PointXYZ p(x, y, z);
        TD_centers.push_back(p);
    }

    return TD_centers;
}

void callback(const sensor_msgs::ImageConstPtr& image_left,const sensor_msgs::ImageConstPtr& image_right) {

  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try
  {
    cv_ptrLeft = cv_bridge::toCvShare(image_left);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrRight;
  try
  {
    cv_ptrRight = cv_bridge::toCvShare(image_right);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  //Calculate the 3D points

  // Converti le immagini a scala di grigi
  Mat gray1, gray2;
  cvtColor(cv_ptrLeft->image, gray1, COLOR_BGR2GRAY);
  cvtColor(cv_ptrRight->image, gray2, COLOR_BGR2GRAY);

  // Crea il rilevatore SIFT
  cv::Ptr<cv::SIFT> sift = cv::SIFT::create();

  // Rileva le caratteristiche SIFT
  std::vector<KeyPoint> keypoints1, keypoints2;
  Mat descriptors1, descriptors2;
  sift->detectAndCompute(gray1, noArray(), keypoints1, descriptors1);
  sift->detectAndCompute(gray2, noArray(), keypoints2, descriptors2);

  // Crea il matcher
  BFMatcher matcher(NORM_L2);
  std::vector<DMatch> matches;
  matcher.match(descriptors1, descriptors2, matches);

  // Ordina le corrispondenze per distanza
  std::sort(matches.begin(), matches.end());

  // Mantiene solo le migliori corrispondenze
  int numGoodMatches = matches.size() * 0.15;
  matches.erase(matches.begin() + numGoodMatches, matches.end());

  // Estrai i punti di corrispondenza
  std::vector<pcl::PointXY> points1, points2;
  for (size_t i = 0; i < matches.size(); i++) {
      pcl::PointXY point1(keypoints1[matches[i].queryIdx].pt.x, keypoints1[matches[i].queryIdx].pt.y);
      points1.push_back(point1);
      pcl::PointXY point2(keypoints2[matches[i].trainIdx].pt.x, keypoints2[matches[i].trainIdx].pt.y);
      points1.push_back(point2);
  }

  std::vector<pcl::PointXYZ> tmp = calculate_TD_Centers(points1, points2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr My_depth_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  for(int i = 0; i < tmp.size(); i++){
    My_depth_cloud->push_back(tmp[i]);
  }

  //Publishing the cloud
  PointCloud2 pub_cloud;
  pcl::toROSMsg(*My_depth_cloud, pub_cloud);
  pub_cloud.header = image_left->header;
  My_depth_cloud_pub.publish(pub_cloud);
  ROS_INFO("[My Depth] Pub final cloud...");
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
    

    ROS_INFO("[My Depth]Starting callback...");
    ros::spin();
    return 0;
}