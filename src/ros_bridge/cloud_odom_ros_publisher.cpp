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

#include "ros_bridge/cloud_odom_ros_publisher.h"
#include "ros_bridge/cloud_odom_ros_subscriber.h" //get extern time

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

#include "image_labelers/diff_helpers/angle_diff.h"
#include "image_labelers/diff_helpers/simple_diff.h"
#include "image_labelers/linear_image_labeler.h"
#include "utils/timer.h"
#include "utils/velodyne_utils.h"
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


void CloudOdomRosPublisher::OnNewObjectReceived(const std::unordered_map<uint16_t, Cloud>& clouds, const int id) {
  (void)id;         // RANDEL: to suppres unused param
  Timer total_timer;
  PointCloudT pcl_cloud;
  ImageToPcl(clouds, pcl_cloud);
  PublishCloud(pcl_cloud);

}

void CloudOdomRosPublisher::ImageToPcl(const std::unordered_map<uint16_t, Cloud>& clouds, PointCloudT& pcl_cloud)
{
  int i = 0;
  for(const auto& kv: clouds){
    const auto& cluster = kv.second;
    PointCloudT pcl_temp;
    PointT min_p;
    PointT max_p;
    for (const auto& point : cluster.points()) {
        PointT p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        p.label = i;
        pcl_temp.push_back(p);
    }
    pcl::getMinMax3D(pcl_temp, min_p, max_p);

    double dif_x = max_p.x-min_p.x;
    double dif_y = max_p.y-min_p.y;
    double dif_z = max_p.z-min_p.z;

    if((dif_x)*(dif_y)<5 || (dif_x*dif_x + dif_y*dif_y + dif_z*dif_z)<3 )
    {
        pcl_cloud+=pcl_temp;
        i++;
    }
  }
}

void CloudOdomRosPublisher::PublishCloud(const PointCloudT& pcl_cloud) {
  sensor_msgs::PointCloud2 cloud2;
  pcl::toROSMsg(pcl_cloud, cloud2);
  cloud2.header.frame_id = _frame_id;
  cloud2.header.stamp = stamps_sub; //From extern stamp_sub from cloud_odom_ros_subscriber.h
  std::cout << "******************************" << "stamps_sub_inpub" << stamps_sub << std::endl;
  //cloud2.header.stamp = ros::Time::now();
  _cloud_pub.publish(cloud2);
}

}  // namespace depth_clustering
