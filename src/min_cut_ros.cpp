#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>

#include <message_filters/subscriber.h>

#include <iostream>
#include <vector>

//including PCL for necessary
#include <pcl/io/pcd_io.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <min_cut_ros/remove_plane.hpp>
#include <min_cut_ros/setting_param.hpp>


ros::Publisher min_cut_publisher;

void min_cut_pub(pcl::PointCloud<pcl::PointXYZ>::Ptr phoxi_points, float sigma, float radius, int non, float sw)
{
  pcl::MinCutSegmentation<pcl::PointXYZ> seg;
  pcl::IndicesPtr indices (new std::vector <int>);
  seg.setInputCloud(phoxi_points);
  seg.setIndices (indices);

  //物体（前景）と背景を切り離す際の，前景の基準となる点を定義
  pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointXYZ pt;
  std::cout << "Please input the x-coordinate of the foreground_points" << std::endl;;
  std::cin >> pt.x;
  std::cout << "Please input the y-coordinate of the foreground_points" << std::endl;
  std::cin >> pt.y;
  std::cout << "Please input the z-coordinate of the foreground_points" << std::endl;
  std::cin >> pt.z;
  foreground_points->points.push_back(pt);
  seg.setForegroundPoints (foreground_points);

  //Min-Cut Based Segmentationに必要な値を設定
  seg.setSigma (sigma);
  seg.setRadius (radius);
  seg.setNumberOfNeighbours (non);
  seg.setSourceWeight (sw);

  std::vector<pcl::PointIndices> clusters;
  seg.extract (clusters);

  pcl::PointCloud<pcl::PointXYZ>::Ptr re_phoxi_points (new pcl::PointCloud<pcl::PointXYZ>);
  re_phoxi_points->points.resize(clusters[1].indices.size());
  //背景から切り離された点群の取得
  for (unsigned int i = 0; i < clusters[1].indices.size(); i++)
    {
      pcl::PointXYZ point;
      point.x = phoxi_points->points[clusters[1].indices[i]].x;
      point.y = phoxi_points->points[clusters[1].indices[i]].y;
      point.z = phoxi_points->points[clusters[1].indices[i]].z;
      re_phoxi_points->points.push_back(point);
    }
  sensor_msgs::PointCloud2 re_phoxi_pointcloud;
  pcl::toROSMsg(*re_phoxi_points, re_phoxi_pointcloud);
  re_phoxi_pointcloud.header.stamp = ros::Time::now();
  re_phoxi_pointcloud.header.frame_id = "/repub";
  min_cut_publisher.publish(re_phoxi_pointcloud);
  ROS_INFO("Publishing pointcloud named %s after processing by Min-Cut Based Segmentation", "/pub_pointcloud");
}

void min_cut_CallBack(const sensor_msgs::PointCloud2::ConstPtr& phoxi_point)
{
  ROS_INFO("Successed subscribing pointcloud named %s", "/sub_pointcloud");
  ROS_INFO("Starting min_cut_CallBack");
  //平面除去用のパラメータの宣言
  float Dist, Prob;
  int remove_plane, MaxIter;
  //Min-Cut Based Segmentationのパラメータの宣言
  float Sigma, Radius, SourceWeight;
  int NoN;
  //各パラメータをroslaunchファイルから取得
  remove_plane_param(remove_plane, MaxIter, Dist, Prob);
  min_cut_param(Sigma, Radius, NoN, SourceWeight);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*phoxi_point, *cloud);
  //平面処理の実行
  Remove_plane(cloud, remove_plane, MaxIter, Dist, Prob);
  //Min-Cut Based Segmentation
  min_cut_pub(cloud, Sigma, Radius, NoN, SourceWeight);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "min_cut_seg");
  ROS_INFO("Initialized");
  ros::NodeHandle nh;
  ros::Subscriber points_sub = nh.subscribe("/sub_pointcloud", 10, min_cut_CallBack);
  min_cut_publisher = nh.advertise<sensor_msgs::PointCloud2>("/pub_pointcloud", 1);
  ros::spin();
  return 0;
}
