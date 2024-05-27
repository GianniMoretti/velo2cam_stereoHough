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
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl_msgs/PointIndices.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <velo2cam_stereoHough/StereoConfig.h>
#include <velo2cam_stereoHough/ClusterCentroids.h>
#include <velo2cam_utils.h>
#include <opencv_apps/CircleArrayStamped.h>

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

pcl::PointCloud<pcl::PointXYZ>::Ptr cumulative_cloud;
ros::Publisher final_pub;
ros::Publisher cumulative_pub;

std_msgs::Header header_;

//DEVE PRENDERE DUE CIRCLESTAMPED
void callback(opencv_apps::CircleArrayStamped circles_stp) {
    if (DEBUG) ROS_INFO("[Stereo] Start processing circles ....");

    images_proc_++;   //Numero di immagini utilizzate?
    header_ = circles_stp.header;

    //Prendere i cerchi e filtrarli e trovare i centri nello spazio 3D.
    //Se tutto va bene aumento images_used_++ e
    //Aggiungo alla comulative cloud
    //Forse prima c'è da fare una rotazione
    //cumulative_cloud->push_back(center_rotated_back);

    // Publishing "cumulative_cloud" (centers found from the beginning)
    if (DEBUG) {
        PointCloud2 cumulative_ros;
        pcl::toROSMsg(*cumulative_cloud, cumulative_ros);
        cumulative_ros.header = circles_stp.header;
        cumulative_pub.publish(cumulative_ros);
        ROS_INFO("[Stereo] %d/%d frames: %ld pts in cloud", images_used_, images_proc_, cumulative_cloud->points.size());
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Compute circles centers
    if (!WARMUP_DONE) {  // Compute clusters from detections in the latest frame
        getCenterClusters(cumulative_cloud, final_cloud, cluster_tolerance_, 1, 1);
    }else{  // Use cumulative information from previous frames
        getCenterClusters(cumulative_cloud, final_cloud, cluster_tolerance_, min_cluster_factor_ * images_used_, images_used_);
        if (final_cloud->points.size() > TARGET_NUM_CIRCLES) {
            getCenterClusters(cumulative_cloud, final_cloud, cluster_tolerance_, 3.0 * images_used_ / 4.0, images_used_);
        }
    }

    // Exit 4: clustering failed
    if (final_cloud->points.size() == TARGET_NUM_CIRCLES) {
        if (save_to_file_) {
            std::vector<pcl::PointXYZ> sorted_centers;
            sortPatternCenters(final_cloud, sorted_centers);
            for (std::vector<pcl::PointXYZ>::iterator it = sorted_centers.begin(); it < sorted_centers.end(); ++it) {
                savefile << it->x << ", " << it->y << ", " << it->z << ", ";
            }
            savefile << cumulative_cloud->width;
        }

        sensor_msgs::PointCloud2 final_ros;
        pcl::toROSMsg(*final_cloud, final_ros);
        final_ros.header = circles_stp.header;

        velo2cam_stereoHough::ClusterCentroids to_send;
        to_send.header = circles_stp.header;
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
        cumulative_cloud->clear();
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


void line_equation(int x1, int y1, int x2, int y2){
    double slope = (y2 - y1) / (x2 - x1);
    double intercept = y1 - slope * x1;
    return {slope, intercept}
}

void intersection_point(double slope1, double intercept1, double slope2, double intercept2){
    double x = (intercept2 - intercept1) / (slope1 - slope2);
    double y = slope1 * x + intercept1
    return {x, y}
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "stereo_pattern");
    ros::NodeHandle nh;        // GLOBAL
    ros::NodeHandle nh_("~");  // LOCAL

    //QUI C'è UN PROBLEMA 
    ros::Subscriber left_circles = nh_.subscribe<opencv_apps::CircleArrayStamped>("circle_detection_left/left_circles", 10, callback);
    ros::Subscriber right_circles = nh_.subscribe<opencv_apps::CircleArrayStamped>("circle_detection_right/right_circles", 10, callback);

    if (DEBUG) {
        cumulative_pub = nh_.advertise<PointCloud2>("cumulative_cloud", 1);
    }

    final_pub = nh_.advertise<velo2cam_stereoHough::ClusterCentroids>("centers_cloud", 1);
    cumulative_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    string csv_name;

    nh.param("delta_width_circles", delta_width_circles_, 0.5);
    nh.param("delta_height_circles", delta_height_circles_, 0.4);
    nh_.param("target_radius_tolerance", target_radius_tolerance_, 0.05); // Era 0.01
    nh_.param("min_centers_found", min_centers_found_, TARGET_NUM_CIRCLES);
    nh_.param("cluster_tolerance", cluster_tolerance_, 0.05);
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

    ros::spin();
    return 0;
}




// def centersFilter(sorted_centers, min_radius, max_radius):
//     #check cernter euclidean min
//     #1. find center of square
//     slope_diag1, intercept_diag1 = line_equation(sorted_centers[0][0], sorted_centers[0][1], sorted_centers[3][0], sorted_centers[3][1])
//     slope_diag2, intercept_diag2 = line_equation(sorted_centers[1][0], sorted_centers[1][1], sorted_centers[2][0], sorted_centers[2][1])

//     # Trova il punto di intersezione delle diagonali
//     intersection_x, intersection_y = intersection_point(slope_diag1, intercept_diag1, slope_diag2, intercept_diag2)

//     radius = sorted([euclidean_distance(intersection_x, intersection_y, c[0], c[1]) for c in sorted_centers])[3]

//     center = np.round((intersection_x, intersection_y)).astype("int")
//     radius = np.round(radius).astype("int")

//     if center[0] > 6000 or center[1] > 6000 or center[0] < 0 or center[1] < 0:
//         return False, (-1, -1), -1

//     if radius > max_radius or radius < min_radius:
//         return False, center, radius
//     else:
//         return True, center, radius

// def sortCenters(centers):
//     if len(centers) > 4:
//         return
//     return sorted(centers, key = lambda x: (x[1], x[0]))

// def depth(circles_right, circles_left):
//     if len(circles_right) != 4 or len(circles_left) != 4:
//         return False, [-1, -1, -1, -1], circles_right, circles_left, (0, 0), 0
    
//     s_circles_right = sortCenters(circles_right)
//     s_circles_left = sortCenters(circles_left)

//     centers_right = [(x,y) for (x, y, r) in s_circles_right]
//     centers_left = [(x,y) for (x, y, r) in s_circles_left]

//     isLeftCentrersValid, center_l, radius_l = centersFilter(centers_left, 100, 150)
//     if not isLeftCentrersValid:
//         return False, [-1, -1, -1, -1], s_circles_right, s_circles_left, (0, 0), 0

//     isRightCentersValid, center_r, center_r = centersFilter(centers_right, 100, 150)
//     if not isRightCentersValid:
//         return False, [-1, -1, -1, -1], s_circles_right, s_circles_left, (0, 0), 0

//     x_l = np.array([p[0] for p in s_circles_right])
//     x_r = np.array([p[0] for p in s_circles_left])

//     disparity = x_r - x_l

//     #Manca il calcolo delle coordinate x e y

//     return True, (fx * baseline) / disparity, s_circles_right, s_circles_left, center_l, radius_l

// def euclidean_distance(x1, y1, x2, y2):
//     # Calcola la distanza euclidea tra due punti nel piano
//     distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
//     return distance
