/* University of Basilicata
 * 
 * Corso di Visione e Percezione
 *
 * Domenico D. Bloisi 
 *
 * domenico.bloisi@unibas.it
 *
 *
 * basic_3d_visualizer
 * 
 * credits
 * Roberto Capobianco
 * Jacopo Serafin
 * Andrea Pennisi
 *
 * This code is provided without any warranty about its usability.
 * It is for educational purposes and should be regarded as such.
 *
 */

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace sensor_msgs;
using namespace message_filters;

void compute3D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
               const cv::Mat& rgbImage,
               const cv::Mat& depthImage);

pcl::visualization::PCLVisualizer* viewer;

void cloud_callback(const ImageConstPtr& depthImage_, const ImageConstPtr& rgbImage_) {
  cv_bridge::CvImagePtr rgbImage =
             cv_bridge::toCvCopy(rgbImage_, sensor_msgs::image_encodings::BGR8);
  cv_bridge::CvImageConstPtr depthImage =
             cv_bridge::toCvShare(depthImage_, sensor_msgs::image_encodings::TYPE_32FC1);

  // Convert from ROS to PCL
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  compute3D(cloud, rgbImage->image, depthImage->image);

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbHandler(cloud);
  viewer->removePointCloud("cloud");
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgbHandler, "cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

  cv::imshow("rgb", rgbImage->image);
  cv::waitKey(30);
}

int main(int argc, char** argv) {
  
  viewer = new pcl::visualization::PCLVisualizer("3d viewer");
    
  viewer->setBackgroundColor(0.33f, 0.33f, 0.33f);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0.0f, 0.0f, 0.0f,
  			    0.0f, 0.0f, 1.0f,
  			    0.0f, -1.0f, 0.0f);

  ros::init(argc, argv, "realsense_r200_3d");

  ros::NodeHandle nh;
  //!!!select the correct topic name for depth data
  message_filters::Subscriber<Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
  //!!!select the correct topic name for color data
  message_filters::Subscriber<Image> rgb_sub(nh, "/camera/rgb/image_rect_color", 1);

  cv::namedWindow("rgb");

  typedef sync_policies::ApproximateTime<Image, Image> syncPolicy;
  Synchronizer<syncPolicy> sync(syncPolicy(10), depth_sub, rgb_sub);
  sync.registerCallback(boost::bind(&cloud_callback, _1, _2));

  while(!viewer->wasStopped()) {
    ros::spinOnce();    
    viewer->spinOnce();
  }
  delete(viewer);

  return 0;
}

void compute3D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
               const cv::Mat& rgbImage,
               const cv::Mat& depthImage)
{
  //camera information based on the Kinect v1 hardware
  //see https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
  //and http://wiki.ros.org/kinect_calibration/technical
  float cx = 319.5f; //optical center x coordinate
  float cy = 239.5f; //optical center y coordinate
  float fx = 525.0f; // focal length x
  float fy = 525.0f; // focal length y 
  
  for(int r = 0; r < rgbImage.rows; r++) {
    for(int c = 0; c < rgbImage.cols; c++) {      
      float d = depthImage.at<float>(r, c);
      if(d != 0 && d < 3500.0f) {
	pcl::PointXYZRGB point;
	point.z = d / 1000.0f;
	point.x = (c - cx) * point.z / fx; 
	point.y = (r - cy) * point.z / fy;
	cv::Vec3b pixel = rgbImage.at<cv::Vec3b>(r, c);
	point.r = pixel[2];
	point.g = pixel[1];
	point.b = pixel[0];
	cloud->points.push_back(point);
      }
    }
  }
}

