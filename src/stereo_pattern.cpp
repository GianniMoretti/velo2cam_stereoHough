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

#define TARGET_NUM_CIRCLES 4

#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl_msgs/PointIndices.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Empty.h>
#include <velo2cam_stereoHough/StereoConfig.h>
#include <velo2cam_stereoHough/ClusterCentroids.h>
#include <velo2cam_utils.h>
#include <opencv_apps/CircleArrayStamped.h>
#include <message_filters/time_synchronizer.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace sensor_msgs;

int images_proc_ = 0, images_used_ = 0;

double delta_width_circles_, delta_height_circles_;
double circle_threshold_;
double target_radius_tolerance_;
double cluster_tolerance_;
int min_centers_found_;
double min_cluster_factor_;
bool WARMUP_DONE = false;
bool skip_warmup_;
bool save_to_file_;
std::ofstream savefile;
int refresh_cloud = 30;


//Questi vanno presi dai parametri  Da cambiare questa cosa
double accept_radius = 80.0;
double delta_filter_accept = 8.0;

// Crea l'oggetto di segmentazione SAC (Sample Consensus)
pcl::SACSegmentation<pcl::PointXYZ> seg;
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

// Configura il metodo RANSAC e il modello sferico

double baseline = 0.12;
boost::array<double, 9> CamIntrMat;

struct line{
    double slope;
    double intercept;
};

cv::Mat PL;
cv::Mat PR;
//cv::Mat T = (cv::Mat_<double>(3, 1) << -0.1166182989733702, 0, 0);
cv::Mat T = (cv::Mat_<double>(3, 1) << -0.1166182989733702, 0, 0);

int fx;
int fy;
int cx;
int cy;

pcl::PointCloud<pcl::PointXYZ>::Ptr cumulative_clouds[4];
pcl::PointCloud<pcl::PointXYZ>::Ptr cumulative_cloud_centroid;

ros::Publisher final_pub;
ros::Publisher cumulative_pub;
ros::Publisher final_debug_pub;

std_msgs::Header header_;

void sortCenters(std::vector<opencv_apps::Point2D>& centers) {
    std::sort(centers.begin(), centers.end(), [](const opencv_apps::Point2D& a, const opencv_apps::Point2D& b) {
        // Primo ordina per il primo elemento, poi per il secondo
        return a.y < b.y;
    });
    if (centers[0].x > centers[1].x){
        opencv_apps::Point2D tmp;
        tmp = centers[0];
        centers[0] = centers[1];
        centers[1] = tmp;
    }
    if (centers[2].x > centers[3].x){
        opencv_apps::Point2D tmp;
        tmp = centers[2];
        centers[2] = centers[3];
        centers[3] = tmp;
    }
}

line line_equation(opencv_apps::Point2D p1, opencv_apps::Point2D p2){
    line l;
    l.slope = (p2.y - p1.y) / (p2.x - p1.x);
    l.intercept = p1.y - l.slope * p1.x;
    return l;
}

opencv_apps::Point2D intersection_point(line l1, line l2){
    opencv_apps::Point2D point;
    point.x = (l2.intercept - l1.intercept) / (l1.slope - l2.slope);
    point.y = l1.slope * point.x + l1.intercept;
    return point;
}

double euclidean_distance(opencv_apps::Point2D p1, opencv_apps::Point2D p2){
    return sqrt(pow((p2.x - p1.x),2) + pow((p2.y - p1.y),2));
}

bool areElementsSimilar(const std::vector<float>& vec, float epsilon) {
    for (size_t i = 1; i < vec.size(); ++i) {
        if (std::fabs(vec[i] - vec[0]) >= epsilon) {
            return false; // Se la differenza è maggiore o uguale a epsilon, gli elementi non sono simili
        }
    }
    return true; // Tutti gli elementi sono simili
}

bool filterCenters(std::vector<opencv_apps::Point2D> centers, double accept_radius){
    //Controllo di plausibilità
    line line1 = line_equation(centers[0], centers[3]);
    line line2 = line_equation(centers[1], centers[2]);

    if (DEBUG) ROS_INFO("line1: %f, %f", line1.slope, line1.intercept);
    if (DEBUG) ROS_INFO("line2: %f, %f", line2.slope, line2.intercept);

    opencv_apps::Point2D square_center = intersection_point(line1, line2);
    ROS_INFO("Squere center: (%f, %f)", square_center.x, square_center.y);
    double radius[4];

    if (DEBUG) ROS_INFO("Calculated radius: ");
    for(int i = 0; i < 4; i++){
        radius[i] = euclidean_distance(centers[i], square_center);
        if (DEBUG) ROS_INFO("radius: %f", radius[i]);
    }

    double max_radius = -1;
    for (int i = 0; i < 4; ++i) {
        if (radius[i] > max_radius) {
            max_radius = radius[i];
        }
    }

    // for (int i = 1; i < 4; ++i) { 
    //     if (std::fabs(radius[i] - radius[0]) >= accept_radius) {
    //         if (DEBUG) ROS_INFO("[Stereo] Centers does not pass the filter, radius: %f", std::fabs(radius[i] - radius[0]));
    //         return false;
    //     }
    // }
    double a = (accept_radius - delta_filter_accept);
    double b = (accept_radius + delta_filter_accept);
    ROS_INFO("[Stereo]a: %f, b: %f", a, b);
    if (max_radius < a || max_radius > b) {
        if (DEBUG) ROS_INFO("[Stereo] Centers does not pass the filter, radius: %f", max_radius);
        return false;
    }
    return true;
}

std::vector<pcl::PointXYZ> calculate_TD_Centers(std::vector<opencv_apps::Point2D> left_centers, std::vector<opencv_apps::Point2D> right_centers){
    std::vector<pcl::PointXYZ> TD_centers;
    
    for(int i = 0; i < 4; i++){
        double z = ((fx * baseline)/(left_centers[i].x - right_centers[i].x));
        //Calcola la coordinata X utilizzando la triangolazione stereo
        double x = z * ((left_centers[i].x - cx) + (right_centers[i].x - cx)) / (2 * fx);
        //Calcola la coordinata Y utilizzando la triangolazione stereo
        //ATTENZIONE: da capire se qui giusto fy
        double y = z * ((left_centers[i].y - cy) + (right_centers[i].y - cy)) / (2 * fy);

        pcl::PointXYZ p(x + 0.06, y, z);
        TD_centers.push_back(p);
    }

    return TD_centers;
}

std::vector<pcl::PointXYZ> calculate_TD_Centers_proj(std::vector<opencv_apps::Point2D> left_centers, std::vector<opencv_apps::Point2D> right_centers){

    std::vector<cv::Point2f> left_centers_f;
    std::vector<cv::Point2f> right_centers_f;

    for(int i = 0; i < 4; i++){
        left_centers_f.push_back(cv::Point2f(left_centers[i].x, left_centers[i].y));
        right_centers_f.push_back(cv::Point2f(right_centers[i].x, right_centers[i].y));
    }

    // ROS_INFO("Points:");
    // for(int i = 0; i < 4; i++){
    //     ROS_INFO("%f, %f", right_centers_f[i].x, left_centers_f[i].y);
    // }

    // std::vector<std::vector<cv::Point2f>> points2d = {left_centers_f, right_centers_f};
    // std::vector<cv::Mat> projection_matrices = {PL, PR};
    // ROS_INFO("Printing PR");
    // for (int i = 0; i < PR.rows; ++i)
    // {
    //     for (int j = 0; j < PR.cols; ++j)
    //     {
    //         ROS_INFO("%f", PR.at<double>(i, j));
    //     }
    //     ROS_INFO(" ");
    // }

    // Variabile di output per i punti 3D
    cv::Mat points3d;
    // Triangolazione dei punti
    cv::triangulatePoints(PL, PR, left_centers_f, right_centers_f, points3d);

    std::vector<pcl::PointXYZ> TD_centers;
    for (int i = 0; i < points3d.cols; i++) {
        cv::Mat x = points3d.col(i);
        //ROS_INFO("%f, %f, %f, %f", x.at<float>(0), x.at<float>(1), x.at<float>(2), x.at<float>(3));
        x /= x.at<float>(3); // Convertire da coordinate omogenee a non omogenee
        pcl::PointXYZ p(x.at<float>(0), x.at<float>(1), x.at<float>(2));
        TD_centers.push_back(p);
    }

    return TD_centers;
}

float randomAdding(){
    if((std::rand() % 2) > 0){
        return (float)(std::rand() % 2) * -1;
    } else {
        return (float)(std::rand() % 2);
    }
}

void callback(const boost::shared_ptr<const opencv_apps::CircleArrayStamped> &left_circles_stp, const boost::shared_ptr<const opencv_apps::CircleArrayStamped> &right_circles_stp) {
    if (DEBUG) {
        ROS_INFO("----------------------------------------------------------------------------------");
        ROS_INFO("[Stereo] Start processing circles ....");
    }

    images_proc_++;   //Numero di immagini utilizzate
    header_ = left_circles_stp->header;

    //Prendere i cerchi e filtrarli e trovare i centri nello spazio 3D.
    std::vector<opencv_apps::Circle> left_circles = left_circles_stp->circles;
    std::vector<opencv_apps::Circle> right_circles = right_circles_stp->circles;

    //Controlla che siano almeno 4 altrimenti esco
    if(left_circles.size() < 4 || right_circles.size() < 4){
        //Avviso di errore
        if (DEBUG) ROS_INFO("[Stereo] Not enoght circle, left: %d, right: %d", left_circles.size(), right_circles.size());
        return;
    }

    std::vector<opencv_apps::Point2D> left_centers;
    std::vector<opencv_apps::Point2D> right_centers;

    for(int i = 0; i < 4; i++){
        opencv_apps::Point2D ptmp_l;
        ptmp_l.x = left_circles[i].center.x + randomAdding();
        ptmp_l.y = left_circles[i].center.y + randomAdding();
        left_centers.push_back(ptmp_l);
        opencv_apps::Point2D ptmp_r;
        ptmp_r.x = right_circles[i].center.x + randomAdding();
        ptmp_r.y = right_circles[i].center.y + randomAdding();
        right_centers.push_back(ptmp_r);
    }

    sortCenters(left_centers);
    sortCenters(right_centers);

    ROS_INFO("Left Centers:");
    for(int i = 0; i < 4; i++){
        ROS_INFO("Center %d: (%f, %f)", i, left_centers[i].x, left_centers[i].y);
    }

    ROS_INFO("Right centers:");
    for(int i = 0; i < 4; i++){
        ROS_INFO("Center %d: (%f, %f)", i,  right_centers[i].x, right_centers[i].y);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if(filterCenters(left_centers, accept_radius)){
        if(filterCenters(right_centers, accept_radius)){
            //I centri sono buoni posso calcolare i punti
            std::vector<pcl::PointXYZ> circle_centers_TD = calculate_TD_Centers(left_centers, right_centers);

            for(int i = 0; i < 4; i++){
                if (DEBUG) ROS_INFO("[Stereo] OK, adding centers to cumulative cloud x: %f, y: %f, z: %f",circle_centers_TD[i].x,circle_centers_TD[i].y,circle_centers_TD[i].z);
                // Forse andrebbe ruotato i punti prima?
                cumulative_clouds[i]->push_back(circle_centers_TD[i]);
                cumulative_cloud_centroid->push_back(circle_centers_TD[i]);
                // if(images_used_ % 10 == 0){
                //     // Esegui la segmentazione
                //     seg.setOptimizeCoefficients(true);
                //     seg.setModelType(pcl::SACMODEL_SPHERE);
                //     seg.setMethodType(pcl::SAC_RANSAC);
                //     seg.setDistanceThreshold(0.8);
                //     seg.setInputCloud(cumulative_clouds[i]);
                //     seg.segment(*inliers, *coefficients);
                // }
                //if(images_used_ % refresh_cloud == 0){
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*cumulative_clouds[i], centroid);
                pcl::PointXYZ centroid_p(centroid[0], centroid[1], centroid[2]);
                if (DEBUG) ROS_INFO("[Stereo] New centroid %d position = x: %f, y: %f, z: %f", i, centroid_p.x, centroid_p.y, centroid_p.z);
                final_cloud->push_back(centroid_p);
                //}
            }   
        }else{
            return;
        }
    }else{
        return;
    }

    images_used_++;

    // Publishing "cumulative_cloud" (centers found from the beginning)
    if (DEBUG) {
        PointCloud2 cumulative_ros;
        pcl::toROSMsg(*cumulative_cloud_centroid, cumulative_ros);
        cumulative_ros.header = left_circles_stp->header;
        cumulative_ros.header.frame_id = "zed2i_left_camera_optical_frame";
        cumulative_pub.publish(cumulative_ros);
        ROS_INFO("[Stereo] %d/%d frames: %ld pts in cloud", images_used_, images_proc_, cumulative_cloud_centroid->points.size());
    }

    // //Compute circles centers
    // if (!WARMUP_DONE) {  // Compute clusters from detections in the latest frame
    //     getCenterClusters(cumulative_cloud_centroid, final_cloud, cluster_tolerance_, 1, 1);
    // }else{  // Use cumulative information from previous frames
    //     getCenterClusters(cumulative_cloud_centroid, final_cloud, cluster_tolerance_, 0.8 * images_used_, images_used_ * 1.2);
    //     if (final_cloud->points.size() > TARGET_NUM_CIRCLES) {
    //         getCenterClusters(cumulative_cloud_centroid, final_cloud, cluster_tolerance_, 3.0 * images_used_ / 4.0, images_used_);
    //     }
    // }

    //clustering failed
    if (final_cloud->points.size() == TARGET_NUM_CIRCLES) {
        if (save_to_file_) {
            std::vector<pcl::PointXYZ> sorted_centers;
            sortPatternCenters(final_cloud, sorted_centers);
            for (std::vector<pcl::PointXYZ>::iterator it = sorted_centers.begin(); it < sorted_centers.end(); ++it) {
                savefile << it->x << ", " << it->y << ", " << it->z << ", ";
            }
            savefile << cumulative_cloud_centroid->width;
        }

        sensor_msgs::PointCloud2 final_ros;
        pcl::toROSMsg(*final_cloud, final_ros);
        final_ros.header = left_circles_stp->header;

        //Pubblichiamo anche la final
        if (DEBUG) {
            PointCloud2 final_debug_ros;
            pcl::toROSMsg(*final_cloud, final_debug_ros);
            final_debug_ros.header = left_circles_stp->header;
            final_debug_ros.header.frame_id = "zed2i_left_camera_optical_frame";
            final_debug_pub.publish(final_debug_ros);
            ROS_INFO("[Stereo] Pub final cloud...");
            ROS_INFO("----------------------------------------------------------------------------------");
        }

        velo2cam_stereoHough::ClusterCentroids to_send;
        // TODO: Da cambiare il frame
        to_send.header = left_circles_stp->header;
        to_send.header.frame_id = "zed2i_left_camera_optical_frame";
        to_send.total_iterations = images_proc_;
        to_send.cluster_iterations = images_used_;
        to_send.cloud = final_ros;
        final_pub.publish(to_send);
        
    }

    if (save_to_file_) {
        savefile << endl;
    }

    // Clear cumulative cloud during warm-up phase
    if (!WARMUP_DONE) {
        cumulative_cloud_centroid->clear();
        for(int i = 0; i < 4; i++){
            cumulative_clouds[i]->clear();
        }  
        images_proc_ = 0;
        images_used_ = 0;
    }
}

void param_callback(velo2cam_stereoHough::StereoConfig &config, uint32_t level) {
    accept_radius = config.accept_radius;
    circle_threshold_ = config.circle_threshold;
    ROS_INFO("[Stereo] New circle threshold: %f", circle_threshold_);
    ROS_INFO("[Stereo] New accept radius: %f", accept_radius);
}

void warmup_callback(const std_msgs::Empty::ConstPtr &msg) {
    WARMUP_DONE = !WARMUP_DONE;
    if (WARMUP_DONE) {
        ROS_INFO("[Stereo] Warm up done, pattern detection started");
    } else {
        ROS_INFO("[Stereo] Detection stopped. Warm up mode activated");
    }
}

void camInfoCallbackLeft(const sensor_msgs::CameraInfoConstPtr &caminfo){
    //Da cambiare metodo di assegnamento
    CamIntrMat = caminfo->K;
    fx = CamIntrMat[0];
    fy = CamIntrMat[4];
    cx = CamIntrMat[2];
    cy = CamIntrMat[5];

    cv::Mat KL = cv::Mat(3, 3, CV_64F, (void*)caminfo->K.elems);
    cv::Mat TL = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    cv::hconcat(KL, TL, PL);
    //PL = cv::Mat(3, 4, CV_64F, (void*)caminfo->P.elems);S
    //ROS_INFO("Printing KL");
    // for (int i = 0; i < KL.rows; ++i)
    // {
    //     for (int j = 0; j < KL.cols; ++j)
    //     {
    //         ROS_INFO("%f", KL.at<double>(i, j));
    //     }
    //     ROS_INFO(" ");
    // }
    // ROS_INFO("Printing PL");
    // for (int i = 0; i < PL.rows; ++i)
    // {
    //     for (int j = 0; j < PL.cols; ++j)
    //     {
    //         ROS_INFO("%f", PL.at<double>(i, j));
    //     }
    //     ROS_INFO(" ");
    // }
}

void camInfoCallbackRight(const sensor_msgs::CameraInfoConstPtr &caminfo){
    //Da cambiare metodo di assegnamento
    cv::Mat KR = cv::Mat(3, 3, CV_64F, (void*)caminfo->K.elems);
    cv::Mat RR = cv::Mat(3, 3, CV_64F, (void*)caminfo->R.elems);
    cv::Mat RTR;
    cv::hconcat(RR, T, RTR);
    PR = KR * RTR;
    //PR = cv::Mat(3, 4, CV_64F, (void*)caminfo->P.elems);
    // ROS_INFO("Printing KR");
    // for (int i = 0; i < KR.rows; ++i)
    // {
    //     for (int j = 0; j < KR.cols; ++j)
    //     {
    //         ROS_INFO("%f", KR.at<double>(i, j));
    //     }
    //     ROS_INFO(" ");
    // }
    // ROS_INFO("Printing RR");
    // for (int i = 0; i < RR.rows; ++i)
    // {
    //     for (int j = 0; j < RR.cols; ++j)
    //     {
    //         ROS_INFO("%f", RR.at<double>(i, j));
    //     }
    //     ROS_INFO(" ");
    // }
    // ROS_INFO("Printing PR");
    // for (int i = 0; i < PR.rows; ++i)
    // {
    //     for (int j = 0; j < PR.cols; ++j)
    //     {
    //         ROS_INFO("%f", PR.at<double>(i, j));
    //     }
    //     ROS_INFO(" ");
    // }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "stereo_pattern");
    ros::NodeHandle nh;        // GLOBAL
    ros::NodeHandle nh_("~");  // LOCAL

    ros::Subscriber cam_info_l = nh.subscribe<sensor_msgs::CameraInfo>("zed2i/zed_node/left_raw/camera_info", 1, camInfoCallbackLeft);
    ros::Subscriber cam_info_r = nh.subscribe<sensor_msgs::CameraInfo>("zed2i/zed_node/right_raw/camera_info", 1, camInfoCallbackRight);

    message_filters::Subscriber<opencv_apps::CircleArrayStamped> left_circles(nh, "circle_detection_left/left_circles", 1);
    message_filters::Subscriber<opencv_apps::CircleArrayStamped> right_circles(nh, "circle_detection_right/right_circles", 1);

    typedef message_filters::sync_policies::ApproximateTime<opencv_apps::CircleArrayStamped, opencv_apps::CircleArrayStamped> AppSync;
    message_filters::Synchronizer<AppSync> sync_(AppSync(100), left_circles, right_circles);
    sync_.registerCallback(boost::bind(&callback, _1, _2));

    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    if (DEBUG) {
        cumulative_pub = nh_.advertise<PointCloud2>("cumulative_cloud", 1);
        final_debug_pub = nh_.advertise<PointCloud2>("final_debug_pub", 1);
    }

    final_pub = nh_.advertise<velo2cam_stereoHough::ClusterCentroids>("centers_cloud", 1);
    cumulative_cloud_centroid = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i= 0; i < 4; i++){
        cumulative_clouds[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    string csv_name;

    nh.param("delta_width_circles", delta_width_circles_, 0.5);
    nh.param("delta_height_circles", delta_height_circles_, 0.4);
    nh_.param("target_radius_tolerance", target_radius_tolerance_, 0.05); // Era 0.01
    nh_.param("min_centers_found", min_centers_found_, TARGET_NUM_CIRCLES);
    nh_.param("cluster_tolerance", cluster_tolerance_, 0.1); // Era 0.05
    nh_.param("min_cluster_factor", min_cluster_factor_, 0.5);
    nh_.param("skip_warmup", skip_warmup_, false);
    nh_.param("save_to_file", save_to_file_, false);
    nh_.param("csv_name", csv_name, "stereo_pattern_" + currentDateTime() + ".csv");

    dynamic_reconfigure::Server<velo2cam_stereoHough::StereoConfig> server;
    dynamic_reconfigure::Server<velo2cam_stereoHough::StereoConfig>::CallbackType f;
    f = boost::bind(param_callback, _1, _2);
    server.setCallback(f);

    ros::Subscriber warmup_sub = nh.subscribe("warmup_switch", 1, warmup_callback);

    if (skip_warmup_) {
        ROS_WARN("Skipping warmup");
        WARMUP_DONE = true;
    }

    // Just for statistics
    if (save_to_file_) {
        ostringstream os;
        os << getenv("HOME") << "/v2c_experiments/" << csv_name;
        if (save_to_file_) {
        if (DEBUG) ROS_INFO("Opening %s", os.str().c_str());
        savefile.open(os.str().c_str());
        savefile << "det1_x, det1_y, det1_z, det2_x, det2_y, det2_z, det3_x, "
                    "det3_y, det3_z, det4_x, det4_y, det4_z, cent1_x, cent1_y, "
                    "cent1_z, cent2_x, cent2_y, cent2_z, cent3_x, cent3_y, "
                    "cent3_z, cent4_x, cent4_y, cent4_z, it" << endl;
        }
    }
    ROS_INFO("[Stereo]Starting callback...");
    ros::spin();
    return 0;
}