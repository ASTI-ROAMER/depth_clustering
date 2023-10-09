// Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#include "depth_clustering/utils/cloud.h"

namespace depth_clustering {

template <class T>
T BytesTo(const std::vector<uint8_t>& data, uint32_t start_idx) {
  const size_t kNumberOfBytes = sizeof(T);
  uint8_t byte_array[kNumberOfBytes];
  // forward bit order (it is a HACK. We do not account for bigendianes)
  for (size_t i = 0; i < kNumberOfBytes; ++i) {
    byte_array[i] = data[start_idx + i];
  }
  T result;
  std::copy(reinterpret_cast<const uint8_t*>(&byte_array[0]),
            reinterpret_cast<const uint8_t*>(&byte_array[kNumberOfBytes]),
            reinterpret_cast<uint8_t*>(&result));
  return result;
}

// void ImageToPcl2(const std::unordered_map<uint16_t, Cloud>& clouds, sensor_msgs::PointCloud2& pcl2_msg)
// {
//   int i = 0;
//   PointCloudT pcl1_cloud;
//   for(const auto& kv: clouds){
//     const auto& cluster = kv.second;
//     PointCloudT pcl_temp;
    
//     PointT min_p;
//     PointT max_p;
//     for (const auto& point : cluster.points()) {
//         PointT p;
//         p.x = point.x();
//         p.y = point.y();
//         p.z = point.z();
//         p.label = i;
//         pcl_temp.push_back(p);
//     }
//     pcl::getMinMax3D(pcl_temp, min_p, max_p);

//     double dif_x = max_p.x-min_p.x;
//     double dif_y = max_p.y-min_p.y;
//     double dif_z = max_p.z-min_p.z;

//     if((dif_x)*(dif_y)<5 || (dif_x*dif_x + dif_y*dif_y + dif_z*dif_z)<3 )
//     {
//         pcl1_cloud+=pcl_temp;
//         i++;
//     }
//   }
// //   sensor_msgs::PointCloud2 cloud2;
//   pcl::toROSMsg(pcl1_cloud, pcl2_msg);
// }

Cloud::Cloud(const Cloud& cloud)
    : _points{cloud.points()},
      _pose(cloud.pose()),
      _sensor_pose(cloud.sensor_pose()) {
  if (!cloud.projection_ptr()) {
    // no need to copy projection, there is none yet
    return;
  }
  // projection is a polymorphic type, we use clone therefore
  auto ptr = cloud.projection_ptr()->Clone();
  _projection = ptr;
}

Cloud::Cloud(const sensor_msgs::PointCloud2& pcl2_msg): 
        _points{}, _pose{}, _sensor_pose{}
{
  uint32_t x_offset = pcl2_msg.fields[0].offset;
  uint32_t y_offset = pcl2_msg.fields[1].offset;
  uint32_t z_offset = pcl2_msg.fields[2].offset;
  uint32_t ring_offset = pcl2_msg.fields[4].offset;

  for (uint32_t point_start_byte = 0, counter = 0;
       point_start_byte < pcl2_msg.data.size();
       point_start_byte += pcl2_msg.point_step, ++counter) {
    RichPoint point;
    point.x() = BytesTo<float>(pcl2_msg.data, point_start_byte + x_offset);
    point.y() = BytesTo<float>(pcl2_msg.data, point_start_byte + y_offset);
    point.z() = BytesTo<float>(pcl2_msg.data, point_start_byte + z_offset);
    point.ring() = BytesTo<uint16_t>(pcl2_msg.data, point_start_byte + ring_offset);
    // point.z *= -1;  // hack
    _points.push_back(point);
  }

}

Cloud& Cloud::operator=(const Cloud& other)
{
  _points = other.points();
  _pose = other.pose();
  _sensor_pose = other.sensor_pose();
  if (!other.projection_ptr()) {
    // no need to copy projection, there is none yet
    return *this;
  }
  // projection is a polymorphic type, we use clone therefore
  auto ptr = other.projection_ptr()->Clone();
  _projection = ptr;
  return *this;
}

std::list<const RichPoint*> Cloud::PointsProjectedToPixel(int row,
                                                          int col) const {
  std::list<const RichPoint*> point_list;
  if (!_projection) {
    return point_list;
  }
  for (const auto& index : _projection->at(row, col).points()) {
    point_list.push_back(&_points[index]);
  }
  return point_list;
}

void Cloud::TransformInPlace(const Pose& pose) {
  for (auto& point : _points) {
    point = pose * point.AsEigenVector();
  }
  // the projection makes no sense anymore after the coords of points changed.
  this->_projection.reset();
}

Cloud::Ptr Cloud::Transform(const Pose& pose) const {
  Cloud cloud_copy(*this);
  cloud_copy.TransformInPlace(pose);
  return make_shared<Cloud>(cloud_copy);
}

void Cloud::SetProjectionPtr(typename CloudProjection::Ptr proj_ptr) {
  _projection = proj_ptr;
}

void Cloud::InitProjection(const ProjectionParams& params) {
  if (_projection) {
    throw std::runtime_error("projection is already initialized");
  }
  _projection = CloudProjection::Ptr(new SphericalProjection(params));
  if (!_projection) {
    fprintf(stderr, "ERROR: failed to initalize projection.\n");
    return;
  }
  _projection = _projection->Clone();
  _projection->InitFromPoints(_points);
}

Cloud::Ptr Cloud::FromImage(const cv::Mat& image,
                            const ProjectionParams& params) {
  CloudProjection::Ptr proj = CloudProjection::Ptr(new RingProjection(params));
  proj->CheckImageAndStorage(image);
  proj->CloneDepthImage(image);
  Cloud cloud;
  for (int r = 0; r < image.rows; ++r) {
    for (int c = 0; c < image.cols; ++c) {
      if (image.at<float>(r, c) < 0.0001f) {
        continue;
      }
      RichPoint point = proj->UnprojectPoint(image, r, c);
      cloud.push_back(point);
      proj->at(r, c).points().push_back(cloud.points().size() - 1);
    }
  }
  cloud.SetProjectionPtr(proj);
  // we cannot share ownership of this cloud with others, so create a new one
  return boost::make_shared<Cloud>(cloud);
}

// this code will be only there if we use pcl
// #ifdef PCL_FOUND

typename pcl::PointCloud<pcl::PointXYZL>::Ptr Cloud::ToPcl() const {
  using pcl::PointXYZL;
  using PclCloud = pcl::PointCloud<PointXYZL>;
  PclCloud pcl_cloud;
  for (const auto& point : _points) {
    PointXYZL pcl_point;
    pcl_point.x = point.x();
    pcl_point.y = point.y();
    pcl_point.z = point.z();
    pcl_point.label = point.ring();
    pcl_cloud.push_back(pcl_point);
  }
  return boost::make_shared<PclCloud>(pcl_cloud);
}

template <>
Cloud::Ptr Cloud::FromPcl(const pcl::PointCloud<pcl::PointXYZL>& pcl_cloud) {
  Cloud cloud;
  for (const auto& pcl_point : pcl_cloud) {
    RichPoint point(pcl_point.x, pcl_point.y, pcl_point.z);
    point.ring() = pcl_point.label;
    cloud.push_back(point);
  }
  return make_shared<Cloud>(cloud);
}

// #endif  // PCL_FOUND

}  // namespace depth_clustering
