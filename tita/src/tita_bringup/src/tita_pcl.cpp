// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "tita_utils/tita_pcl.hpp"

namespace tita_pcl
{

void voxel_grid(CloudPtr cloud, float voxel_size)
{
  pcl::VoxelGrid<PointType> voxel;
  voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel.setInputCloud(cloud);

  CloudPtr output(new PointCloudType);
  voxel.filter(*output);
  cloud->swap(*output);
}

void pass_through_box(CloudPtr incloud, CloudPtr outcloud, double * limited)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(incloud);

  pass.setFilterFieldName("x");
  pass.setFilterLimits(limited[0], limited[1]);
  pass.filter(*temp_cloud);

  pass.setInputCloud(temp_cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(limited[2], limited[3]);
  pass.filter(*temp_cloud);

  pass.setInputCloud(temp_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(limited[4], limited[5]);
  pass.filter(*outcloud);
}

}  // namespace tita_pcl
