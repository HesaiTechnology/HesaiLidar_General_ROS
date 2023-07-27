/******************************************************************************
 * Copyright 2018 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef INCLUDE_POINT_TYPES_H_
#define INCLUDE_POINT_TYPES_H_

#include <pcl/point_types.h>

struct PointXYZIT {
  PCL_ADD_POINT4D   //添加pcl里xyz
  float intensity;
  double timestamp;
  uint16_t ring;                   ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned,确保定义新类型点云内存与SSE对齐
} EIGEN_ALIGN16;                   // 强制SSE填充以正确对齐内存

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        double, timestamp, time)(uint16_t, ring, ring))

typedef PointXYZIT PPoint;
typedef pcl::PointCloud<PPoint> PPointCloud;

#endif  // INCLUDE_POINT_TYPES_H_
