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
#include <message_filters/time_synchronizer.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <string>
#include <std_msgs/Empty.h>

using namespace cv;
using namespace std;
using namespace sensor_msgs;

float baseline = 0.12f;

boost::array<double, 9> CamIntrMat;
cv::Mat K;
int camera_width;
int camera_height;

ros::Publisher left_camera_lidar_point;

ros::Subscriber cam_info_sub;
ros::Subscriber cam_img_sub;

ros::Publisher My_depth_cloud_pub;

//static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms 

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


//TODO: da mettere string per bene
string frame_input;
string frame_output;

double mapValue(double x, double input_min, double input_max, double output_min, double output_max) {
    return output_min + ((x - input_min) / (input_max - input_min)) * (output_max - output_min);
}

cv::Scalar getColor(float x, float minX, float maxX) {
    // Normalizza la coordinata X
    float normalizedX = (x - minX) / (maxX - minX);
    //ROS_INFO("NORM: %f", normalizedX);
    float r, g, b;

    if (normalizedX < 0.5f) {
        // Dal blu al verde
        r = 0.0f;
        g = 2.0f * normalizedX;
        b = 1.0f - 2.0f * normalizedX;
    } else {
        // Dal verde al rosso
        r = 2.0f * (normalizedX - 0.5f);
        g = 1.0f - 2.0f * (normalizedX - 0.5f);
        b = 0.0f;
    }

    return cv::Scalar(r * 255, g * 255, b * 255);
}

void callback(const sensor_msgs::ImageConstPtr& image_left, const PointCloud2::ConstPtr& cloud_msg) {

    ROS_INFO("[lidar_to_camera_view]Calculating lidar point in camera....");
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvCopy(image_left, "bgr8");
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("[lidar_to_camera_view]cv_bridge exception: %s", e.what());
        return;
    }

    Mat img1 = cv_ptrLeft->image; 
    int count = cloud_msg->width * cloud_msg->height;

    ROS_INFO("[lidar_to_camera_view]Cloud size : %i", count);

    //Approccio senza utilizzare TF

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // // Converti da PointCloud2 ROS a PointCloud PCL
    // pcl::fromROSMsg(*cloud_msg, *cloud);

    // // Matrici di rotazione
    // Eigen::Matrix3f R_x;
    // R_x << 1, 0, 0,
    //     0, cos(roll_camera), -sin(roll_camera),
    //     0, sin(roll_camera), cos(roll_camera);

    // Eigen::Matrix3f R_y;
    // R_y << cos(pitch_camera), 0, sin(pitch_camera),
    //     0, 1, 0,
    //     -sin(pitch_camera), 0, cos(pitch_camera);

    // Eigen::Matrix3f R_z;
    // R_z << cos(yaw_camera), -sin(yaw_camera), 0,
    //     sin(yaw_camera), cos(yaw_camera), 0,
    //     0, 0, 1;

    // // Matrice di rotazione combinata
    // Eigen::Matrix3f R = R_z * R_y * R_x;

    // // Matrice di rototraslazione
    // Eigen::Matrix4f camera_rotated = Eigen::Matrix4f::Identity();
    // camera_rotated.block<3,3>(0,0) = R;  // Inserisci la matrice di rotazione
    // camera_rotated(0,3) = 0;  // Traslazione lungo x
    // camera_rotated(1,3) = 0;  // Traslazione lungo y
    // camera_rotated(2,3) = 0;  // Traslazione lungo z

    // // Matrici di rotazione
    // R_x << 1, 0, 0,
    //     0, cos(roll), -sin(roll),
    //     0, sin(roll), cos(roll);

    // R_y << cos(pitch), 0, sin(pitch),
    //     0, 1, 0,
    //     -sin(pitch), 0, cos(pitch);

    // R_z << cos(yaw), -sin(yaw), 0,
    //     sin(yaw), cos(yaw), 0,
    //     0, 0, 1;

    // // Matrice di rotazione combinata
    // R = R_z * R_y * R_x;

    // // Matrice di rototraslazione
    // Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    // transform.block<3,3>(0,0) = R;  // Inserisci la matrice di rotazione
    // transform(0,3) = tx;  // Traslazione lungo x
    // transform(1,3) = ty;  // Traslazione lungo y
    // transform(2,3) = tz;  // Traslazione lungo z

    // Eigen::Matrix4f finall_trasform = Eigen::Matrix4f::Identity();
    // finall_trasform = camera_rotated * transform;

    // // Creazione di una point cloud per i dati trasformati
    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // // Applicazione della matrice di trasformazione
    // pcl::transformPointCloud(*cloud, *transformed_cloud, finall_trasform);

    //Usando TF

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    //Converti da PointCloud2 ROS a PointCloud PCL
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // Creiamo un listener per ottenere le trasformazioni tf
    tf::TransformListener listener;

    // Aspetta che la trasformazione sia disponibile
    try {
        listener.waitForTransform(frame_output, frame_input, ros::Time(0), ros::Duration(3.0));
    } catch (tf::TransformException& ex) {
        ROS_ERROR("Errore durante l'attesa della trasformazione: %s", ex.what());
        return;
    }

    // Creazione di una point cloud per i dati trasformati
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Applica la trasformazione da source_frame a target_frame
    try {
        pcl_ros::transformPointCloud(frame_input, *cloud, *transformed_cloud, listener);
        ROS_INFO("Nuvola di punti trasformata con successo!");
    } catch (tf::TransformException& ex) {
        ROS_ERROR("Errore durante la trasformazione della nuvola di punti: %s", ex.what());
        return;
    }

    ROS_INFO("[lidar_to_camera_view]Point trasformation");
    //Publishing the cloud
    PointCloud2 pub_cloud;
    pcl::toROSMsg(*transformed_cloud, pub_cloud);
    pub_cloud.header = image_left->header;
    pub_cloud.header.frame_id = "zed2i_left_camera_frame";
    My_depth_cloud_pub.publish(pub_cloud);
    ROS_INFO("[lidar_to_camera_view]Pub trasformed cloud...");

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

 // Trova l'elemento con la minima coordinata z
    cv::Point3d minElement = *std::min_element(image_points.begin(), image_points.end(), 
                                        [](const cv::Point3d& a, const cv::Point3d& b) {
                                            return a.z < b.z;
                                        });

    // Trova l'elemento con la massima coordinata z
    cv::Point3d maxElement = *std::max_element(image_points.begin(), image_points.end(), 
                                        [](const cv::Point3d& a, const cv::Point3d& b) {
                                            return a.z < b.z;
                                        });

    ROS_INFO("min: %f, max: %f", minElement.z ,maxElement.z);

    for (const auto& pt : image_points) {
        //int color = (int)mapValue(pt.z, 0, 30, 255, 100);
        //Devi aggiungere dei controlli per vedere che stia nell'immagine
        cv::Point2d p (pt.x, pt.y);
        cv::Scalar c = getColor(pt.z, minElement.z, maxElement.z);
        //ROS_INFO("color: %f, %f, %f", c[0], c[1], c[2]);
        cv::circle(img1, p, 3, c, -1);
    }
    ROS_INFO("[lidar_to_camera_view]Pub lidar points image");
    sensor_msgs::ImagePtr ros_left_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img1).toImageMsg();
    left_camera_lidar_point.publish(ros_left_image);
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
    ros::init(argc, argv, "lidar_to_cameraview");
    ros::NodeHandle nh;        // GLOBAL
    ros::NodeHandle nh_("~");  // LOCAL

    cam_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("zed2i/zed_node/left/camera_info", 1, camInfoCallback);

    nh_.param("roll_camera", roll_camera, 0.0f);
    nh_.param("pitch_camera", pitch_camera, 0.0f);
    nh_.param("yaw_camera", yaw_camera, 0.0f);
    nh_.param("frame_input", frame_input, string(""));
    nh_.param("frame_output", frame_output, string(""));

    left_camera_lidar_point = nh_.advertise<sensor_msgs::Image>("left_camera_lidar_point", 1);

    message_filters::Subscriber<sensor_msgs::Image> left_image(nh, "image_left", 1);
    message_filters::Subscriber<PointCloud2> velo_points(nh, "points", 1);

    My_depth_cloud_pub = nh_.advertise<PointCloud2>("Velodyne_trasformed_cloud", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, PointCloud2> AppSync;
    message_filters::Synchronizer<AppSync> sync_(AppSync(100), left_image, velo_points);
    sync_.registerCallback(boost::bind(&callback, _1, _2));



    ROS_INFO("[lidar_to_cameraview]Starting callback...");
    ros::spin();
    return 0;
}