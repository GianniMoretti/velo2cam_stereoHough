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
#include <dynamic_reconfigure/server.h>
#include <algorithm>
#include <fstream>  // Libreria per gestire file
#include <opencv_apps/CircleArrayStamped.h>
#include <velo2cam_stereoHough/ClusterCentroids.h>

using namespace cv;
using namespace std;
using namespace sensor_msgs;

// Nome del file
std::string filename = "datiIoU.csv";

// Apri un file CSV in modalità scrittura
std::ofstream file;

float baseline = 0.12f;

boost::array<double, 9> CamIntrMat;
cv::Mat K;
int camera_width;
int camera_height;

ros::Publisher left_camera_lidar_point;

ros::Subscriber cam_info_sub;
ros::Subscriber cam_img_sub;

ros::Publisher My_depth_cloud_pub;
//-0.330524 0.273133 -0.986594 -0.116133 0.00141425 -0.0183019

// Definisci gli angoli di rotazione in radianti
float roll = -0.1161339;   // Rotazione intorno all'asse x
float pitch = 0.00141425;  // Rotazione intorno all'asse y
float yaw = -0.0183019;    // Rotazione intorno all'asse z

// Definisci le traslazioni
float tx = -0.330524;//0.330524;  // Traslazione lungo l'asse x
float ty = 0.003133;  // Traslazione lungo l'asse y
float tz = -0.986594;  // Traslazione lungo l'asse z

float roll_camera = 0;
float pitch_camera = 0;
float yaw_camera = 0;

int countCycle = 0;

void callback(const boost::shared_ptr<const opencv_apps::CircleArrayStamped> &left_circles_stp, const velo2cam_stereoHough::ClusterCentroids::ConstPtr sensor1_centroids) {

    if(countCycle){
        ROS_INFO("[saveImg]Calculating lidar point in camera....");
        
        //Acquisizione centri immagine 
        std::vector<opencv_apps::Circle> left_circles = left_circles_stp->circles;
        for(int i = 0; i < 4; i++){
            //capire come e dove salvare
            file << left_circles[i].center.x << ","
            << left_circles[i].center.y << ","
            << left_circles[i].radius << ",";
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Converti da PointCloud2 ROS a PointCloud PCL
        pcl::fromROSMsg(sensor1_centroids->cloud, *cloud);

        // Matrici di rotazione
        Eigen::Matrix3f R_x;
        R_x << 1, 0, 0,
            0, cos(roll_camera), -sin(roll_camera),
            0, sin(roll_camera), cos(roll_camera);

        Eigen::Matrix3f R_y;
        R_y << cos(pitch_camera), 0, sin(pitch_camera),
            0, 1, 0,
            -sin(pitch_camera), 0, cos(pitch_camera);

        Eigen::Matrix3f R_z;
        R_z << cos(yaw_camera), -sin(yaw_camera), 0,
            sin(yaw_camera), cos(yaw_camera), 0,
            0, 0, 1;

        // Matrice di rotazione combinata
        Eigen::Matrix3f R = R_z * R_y * R_x;

        // Matrice di rototraslazione
        Eigen::Matrix4f camera_rotated = Eigen::Matrix4f::Identity();
        camera_rotated.block<3,3>(0,0) = R;  // Inserisci la matrice di rotazione
        camera_rotated(0,3) = 0;  // Traslazione lungo x
        camera_rotated(1,3) = 0;  // Traslazione lungo y
        camera_rotated(2,3) = 0;  // Traslazione lungo z

        // Matrici di rotazione
        R_x << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll);

        R_y << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch);

        R_z << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;

        // Matrice di rotazione combinata
        R = R_z * R_y * R_x;

        // Matrice di rototraslazione
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3,3>(0,0) = R;  // Inserisci la matrice di rotazione
        transform(0,3) = tx;  // Traslazione lungo x
        transform(1,3) = ty;  // Traslazione lungo y
        transform(2,3) = tz;  // Traslazione lungo z

        Eigen::Matrix4f finall_trasform = Eigen::Matrix4f::Identity();
        finall_trasform = camera_rotated * transform;

        // Creazione di una point cloud per i dati trasformati
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Applicazione della matrice di trasformazione
        pcl::transformPointCloud(*cloud, *transformed_cloud, finall_trasform);

        std::vector<cv::Point3d> image_points;
        for (const auto& point : transformed_cloud->points) {
            if (point.z > 0){
                cv::Mat pt_3d = (cv::Mat_<double>(4, 1) << point.x, point.y, -point.z, 1);
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
            cv::Point2d p (pt.x, pt.y);
            file << pt.x << "," << pt.y << "\n";
        }

        //Qua devi fare la magia


        ROS_INFO("[saveImg]Pub lidar points image");
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

    file.open(filename);

    // Controllo se il file si è aperto correttamente
    if (!file.is_open()) {
        std::cerr << "Errore nell'apertura del file!" << std::endl;
        return 1;
    }

    nh_.param("roll_camera", roll_camera, 0.0f);
    nh_.param("pitch_camera", pitch_camera, 0.0f);
    nh_.param("yaw_camera", yaw_camera, 0.0f);

    left_camera_lidar_point = nh_.advertise<sensor_msgs::Image>("left_camera_lidar_point", 1);

    message_filters::Subscriber<opencv_apps::CircleArrayStamped> left_circle(nh, "image_left", 1);
    message_filters::Subscriber<velo2cam_stereoHough::ClusterCentroids> velo_circle_points(nh, "points", 1);

    typedef message_filters::sync_policies::ApproximateTime<opencv_apps::CircleArrayStamped, velo2cam_stereoHough::ClusterCentroids> AppSync;
    message_filters::Synchronizer<AppSync> sync_(AppSync(100), left_circle, velo_circle_points);
    sync_.registerCallback(boost::bind(&callback, _1, _2));

    ROS_INFO("[saveImg]Starting callback...");
    ros::spin();

    // Chiudi il file
    file.close();

    std::cout << "Dati salvati con successo in " << filename << std::endl;

    return 0;
}