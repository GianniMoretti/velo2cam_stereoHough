#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
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
#include <dynamic_reconfigure/server.h>
#include <algorithm>
#include <fstream>  // Libreria per gestire file
#include <opencv_apps/CircleArrayStamped.h>
#include <velo2cam_stereoHough/ClusterCentroids.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace cv;
using namespace std;
using namespace sensor_msgs;

// Nome del file
std::string filename_camera = "/home/slam/catkin_ws_lidar/CentriLidarCamera/centriCamera.csv";
std::string filename_lidar = "/home/slam/catkin_ws_lidar/CentriLidarCamera/centriLidar.csv";

// Apri un file CSV in modalità scrittura
std::ofstream file_camera;
std::ofstream file_lidar;

float baseline = 0.12f;

boost::array<double, 9> CamIntrMat;
cv::Mat K;
int camera_width;
int camera_height;

ros::Subscriber cam_info_sub;
ros::Subscriber cam_img_sub;

int countCycle = 0;
bool finito = false;
int itBet = 10;
int numOfSample = 20;

void callback(const boost::shared_ptr<const opencv_apps::CircleArrayStamped> &left_circles_stp, const PointCloud2::ConstPtr& cloud_msg) {
    if(countCycle % itBet == 0){
        ROS_INFO("[saveImg]Calculating lidar point in camera....");
        
        //Cosi non va bene!!
        //Acquisizione centri immagine 
        std::vector<opencv_apps::Circle> left_circles = left_circles_stp->circles;
        for(int i = 0; i < 4; i++){
            //capire come e dove salvare
            file_camera << int(countCycle / itBet) << "," 
            << left_circles[i].center.x << ","
            << left_circles[i].center.y << ","
            << left_circles[i].radius << "\n";
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Converti da PointCloud2 ROS a PointCloud PCL
        pcl::fromROSMsg(*cloud_msg, *cloud);

        //Usando TF
        // Creiamo un listener per ottenere le trasformazioni tf
        tf::TransformListener listener;
        tf::StampedTransform transform;

        // Aspetta che la trasformazione sia disponibile
        try {
            listener.waitForTransform("zed2i_left_camera_optical_frame", "velodyne", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("zed2i_left_camera_optical_frame", "velodyne", ros::Time(0), transform);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("Errore durante l'attesa della trasformazione: %s", ex.what());
            return;
        }

        tf::Transform inverseTransform = transform.inverse();

        // Creazione di una point cloud per i dati trasformati
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Applica la trasformazione da source_frame a target_frame
        try {
            //pcl_ros::transformPointCloud(frame_output, *inverseTransform, *cloud, *transformed_cloud);
            //pcl_ros::transformPointCloud(frame_input, *cloud, *transformed_cloud, listener);
            pcl_ros::transformPointCloud(*cloud, *transformed_cloud, transform);
            ROS_INFO("Nuvola di punti trasformata con successo!");
        } catch (tf::TransformException& ex) {
            ROS_ERROR("Errore durante la trasformazione della nuvola di punti: %s", ex.what());
            return;
        }

        std::vector<cv::Point3d> image_points;
        for (const auto& point : transformed_cloud->points) {
            if (point.z > 0){
                cv::Mat pt_3d = (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1);
                cv::Mat pt_2d = K * pt_3d;
                // Normalizza per ottenere le coordinate (x, y)
                pt_2d /= pt_2d.at<double>(2);
                //ROS_INFO("point2d: (%f,%f)", pt_2d.at<double>(0), pt_2d.at<double>(1));
                if(pt_2d.at<double>(0) >= 0 && pt_2d.at<double>(0) < camera_width)
                {
                    if(pt_2d.at<double>(1) >= 0 && pt_2d.at<double>(1) < camera_height){
                        image_points.push_back(cv::Point3d(pt_2d.at<double>(0), pt_2d.at<double>(1), abs(point.z)));
                    }
                }
            }
        }

        for (const auto& pt : image_points) {
            file_lidar << int(countCycle / itBet) << "," << pt.x << "," << pt.y << "\n";
        }

        ROS_INFO("[saveImg]Pub lidar points image");
    }
    ROS_INFO("[saveImg]Cycle : %i", countCycle);
    countCycle++;
    if(countCycle > itBet * numOfSample){

        // Chiudi il file
        if(finito == false){
            file_camera.close();
            file_lidar.close();
        }
        finito = true;

        std::cout << "Dati salvati con successo."  << std::endl;

    }
}

void camInfoCallback(const sensor_msgs::CameraInfoConstPtr &caminfo){
    //Da cambiare metodo di assegnamento
    CamIntrMat = caminfo->K;
    K = (cv::Mat_<double>(3, 4) << 
    CamIntrMat[0], 0, CamIntrMat[2], 0,
    0, CamIntrMat[4], CamIntrMat[5], 0,
    0, 0, 1, 0);
    camera_width = caminfo->width;
    camera_height = caminfo->height;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "saveImg");
    ros::NodeHandle nh;        // GLOBAL
    ros::NodeHandle nh_("~");  // LOCAL

    cam_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("zed2i/zed_node/left/camera_info", 1, camInfoCallback);

    file_camera.open(filename_camera);
    file_lidar.open(filename_lidar);

    // Controllo se il file si è aperto correttamente
    if (!file_camera.is_open()) {
        std::cerr << "Errore nell'apertura del file!" << std::endl;
        return 1;
    }
        // Controllo se il file si è aperto correttamente
    if (!file_lidar.is_open()) {
        std::cerr << "Errore nell'apertura del file!" << std::endl;
        return 1;
    }

    file_camera << "index, x cerchio,y cerchio,raggio\n";
    file_lidar << "index, x cerlidar,y cerlidar\n";

    message_filters::Subscriber<opencv_apps::CircleArrayStamped> left_circle(nh, "circle_detection_left/left_circles", 1);
    message_filters::Subscriber<PointCloud2> velo_circle_points(nh, "lidar_pattern_0/cumulative_cloud", 1);

    typedef message_filters::sync_policies::ApproximateTime<opencv_apps::CircleArrayStamped, PointCloud2> AppSync;
    message_filters::Synchronizer<AppSync> sync_(AppSync(500), left_circle, velo_circle_points);
    sync_.registerCallback(boost::bind(&callback, _1, _2));

    ROS_INFO("[saveImg]Starting callback...");
    ros::spin();

    return 0;
}