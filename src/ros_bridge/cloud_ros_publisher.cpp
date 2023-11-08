// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "depth_clustering/ros_bridge/cloud_ros_publisher.h"
#include "depth_clustering/ros_bridge/cloud_odom_ros_subscriber.h" //get extern time

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>

#include <opencv4/opencv2/core/cuda.hpp>
// #include <opencv4/opencv2/cudaarithm.hpp>
// #include <opencv4/opencv2/cudaimgproc.hpp>

#include <algorithm>
#include <memory>
#include <cmath>

#include "depth_clustering/image_labelers/diff_helpers/angle_diff.h"
#include "depth_clustering/image_labelers/diff_helpers/simple_diff.h"
#include "depth_clustering/image_labelers/linear_image_labeler.h"
#include "depth_clustering/utils/timer.h"
#include "depth_clustering/utils/velodyne_utils.h"
#include "pcl/common/common.h"

#include <ros/console.h>

#include <iostream> //!!!

namespace depth_clustering {

using std::abs;

using cv::Mat;
using cv::DataType;
using std::to_string;
using time_utils::Timer;
using sensor_msgs::PointCloud2;
using sensor_msgs::PointCloud2ConstPtr;


void CloudRosPublisher::OnNewObjectReceived(const Cloud& cloud, const int id) {
  (void)id;         // RANDEL: to suppres unused param
  Timer total_timer;
  PointCloudT pcl_cloud;
  PointsToPcl(cloud, pcl_cloud);
  // ImageToPcl(cloud, pcl_cloud);
  PublishCloud(pcl_cloud);

}

void CloudRosPublisher::PointsToPcl(const Cloud& cloud, PointCloudT& pcl_cloud)
{
  for (const auto& p_rp: cloud.points()){
    PointT p;
    p.x = p_rp.x();
    p.y = p_rp.y();
    p.z = p_rp.z();
    p.label = 0;

    pcl_cloud.push_back(p);
  }
}

void CloudRosPublisher::ImageToPcl(const Cloud& cloud, PointCloudT& pcl_cloud)
{
  const cv::Mat& depth_img =  cloud.projection_ptr()->depth_image();      // calling it is too long
  PointT p;

  for (int row = 0; row < int(cloud.projection_ptr()->rows()); ++row) {
      for (int col = 0; col < int(cloud.projection_ptr()->cols()); ++col) {
        if (depth_img.at<float>(row, col) != 0.0){
        //   RichPoint point = cloud.projection_ptr()->UnprojectPoint(depth_img, row, col);
          PointT p;
          const auto& point_container = cloud.projection_ptr()->at(row, col);
          if (point_container.IsEmpty()) {
            // fprintf(stderr, "RANDEL: point container empty, size=%lu\n", point_container.points().size());
            // this is ok, just continue, nothing interesting here, no points.
            continue;
          } else{
            // fprintf(stderr, "RANDEL PC not empty size=%lu:\n", point_container.points().size());
            for (const auto& p_idx: point_container.points() ){
                // fprintf(stderr, "idx: %lu", p_idx);
                const RichPoint& pp = cloud.at(p_idx);
                p.x = pp.x();
                p.y = pp.y();
                p.z = pp.z();
                p.label = 0;
                pcl_cloud.push_back(p);
                
            }
            // fprintf(stderr, "\n\n");
          }
          
          // PointT p;
          // p.x = point.x();
          // p.y = point.y();
          // p.z = point.z();
          // // p.label = i;
          // p.label = 0;
          // pcl_cloud.push_back(p);
        }
        
        
      }
  }
  // fprintf(stderr, "RANDEL: orig size: %lu pcl size%lu", cloud.points().size(), pcl_cloud.size());

}

void CloudRosPublisher::PublishCloud(const PointCloudT& pcl_cloud) {
  sensor_msgs::PointCloud2 cloud2;
  pcl::toROSMsg(pcl_cloud, cloud2);
  cloud2.header.frame_id = _frame_id;
  cloud2.header.stamp = stamps_sub; //From extern stamp_sub from cloud_odom_ros_subscriber.h
  std::cout << "******************************" << "stamps_sub_inpub" << stamps_sub << std::endl;
  //cloud2.header.stamp = ros::Time::now();
  _cloud_pub.publish(cloud2);
}

}  // namespace depth_clustering
