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
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl_msgs/PointIndices.h>
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

using namespace std;
using namespace sensor_msgs;

int images_proc_ = 0, images_used_ = 0;

double delta_width_circles_, delta_height_circles_;
double circle_threshold_;
double target_radius_tolerance_;
double cluster_tolerance_;
int min_centers_found_;
double min_cluster_factor_;
bool WARMUP_DONE = true;
bool skip_warmup_;
bool save_to_file_;
std::ofstream savefile;

//Questi vanno presi dai parametri
double accep_radius = 40;
double offset_accept_radius = 10; 
double baseline = 0.12;

boost::array<double, 9> CamIntrMat;

struct line{
    double slope;
    double intercept;
};

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
        if (a.x == b.x) {
            return a.y < b.y;
        }
        return a.x < b.x;
    });
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

bool filterCenters(std::vector<opencv_apps::Point2D> centers, double accep_radius, double offset){
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

    //Magari aggiungere anche un controllo sulla posizione del centro?
    if(max_radius > accep_radius + offset || max_radius < accep_radius - offset){
        if (DEBUG) ROS_INFO("[Stereo] Centers does not pass the filter, radius: %f", max_radius);
        return false;
    }else{
        return true;
    }
}

std::vector<pcl::PointXYZ> calculate_TD_Centers(std::vector<opencv_apps::Point2D> left_centers, std::vector<opencv_apps::Point2D> right_centers){
    std::vector<pcl::PointXYZ> TD_centers;
    for(int i = 0; i < 4; i++){
        double z = ((fx * baseline)/(left_centers[i].x - right_centers[i].x));
        //Calcola la coordinata X utilizzando la triangolazione stereo
        double x = z * ((left_centers[i].x - cx) + (right_centers[i].x - cx)) / (2 * fx);
        //Calcola la coordinata Y utilizzando la triangolazione stereo
        //ATTENZIONE: da capire se qui giusto fy
        double y = z * ((left_centers[i].y - cy) + (right_centers[i].y - cy)) / (2 * fx);

        pcl::PointXYZ p(x, y, z);
        TD_centers.push_back(p);
    }

    return TD_centers;
}

void callback(const boost::shared_ptr<const opencv_apps::CircleArrayStamped> &left_circles_stp, const boost::shared_ptr<const opencv_apps::CircleArrayStamped> &right_circles_stp) {
    if (DEBUG) ROS_INFO("[Stereo] Start processing circles ....");

    images_proc_++;   //Numero di immagini utilizzate
    header_ = left_circles_stp->header;

    for(int i = 0; i < 4; i++){
        ROS_INFO("Center %d: (%f, %f)", i,  left_circles_stp->circles[i].center.x, left_circles_stp->circles[i].center.y);
    }

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
        left_centers.push_back(left_circles[i].center);
        right_centers.push_back(right_circles[i].center);
    }

    sortCenters(left_centers);
    sortCenters(right_centers);

    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if(filterCenters(left_centers, accep_radius, offset_accept_radius)){
        if(filterCenters(right_centers, accep_radius, offset_accept_radius)){
            //I centri sono buoni posso calcolare i punti
            std::vector<pcl::PointXYZ> circle_centers_TD = calculate_TD_Centers(left_centers, right_centers); 

            for(int i = 0; i < 4; i++){
                if (DEBUG) ROS_INFO("[Stereo] OK, adding centers to cumulative cloud");
                // Forse andrebbe ruotato i punti prima?
                cumulative_clouds[i]->push_back(circle_centers_TD[i]);
                cumulative_cloud_centroid->push_back(circle_centers_TD[i]);

                pcl::PointXYZ centroid;
                pcl::computeCentroid(*cumulative_clouds[i], centroid);
                final_cloud->push_back(centroid);
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
            final_debug_pub.publish(final_debug_ros);
            ROS_INFO("[Stereo] Pub final cloud...");
        }

        velo2cam_stereoHough::ClusterCentroids to_send;
        to_send.header = left_circles_stp->header;
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
    circle_threshold_ = config.circle_threshold;
    ROS_INFO("[Stereo] New circle threshold: %f", circle_threshold_);
}

void warmup_callback(const std_msgs::Empty::ConstPtr &msg) {
    WARMUP_DONE = !WARMUP_DONE;
    if (WARMUP_DONE) {
        ROS_INFO("[Stereo] Warm up done, pattern detection started");
    } else {
        ROS_INFO("[Stereo] Detection stopped. Warm up mode activated");
    }
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
    ros::init(argc, argv, "stereo_pattern");
    ros::NodeHandle nh;        // GLOBAL
    ros::NodeHandle nh_("~");  // LOCAL

    //Controllare perchè potrebbe chiamarsi diversamente
    ros::Subscriber cam_info = nh.subscribe<sensor_msgs::CameraInfo>("zed2i/zed_node/left/camera_info", 1, camInfoCallback);

    message_filters::Subscriber<opencv_apps::CircleArrayStamped> left_circles(nh, "circle_detection_left/left_circles", 1);
    message_filters::Subscriber<opencv_apps::CircleArrayStamped> right_circles(nh, "circle_detection_right/right_circles", 1);

    typedef message_filters::sync_policies::ApproximateTime<opencv_apps::CircleArrayStamped, opencv_apps::CircleArrayStamped> AppSync;
    message_filters::Synchronizer<AppSync> sync_(AppSync(100), left_circles, right_circles);
    sync_.registerCallback(boost::bind(&callback, _1, _2));

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