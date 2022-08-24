/*
Copyright (C) 2022 Hongkai Ye (kyle_yeh@163.com)
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/
#ifndef _OCC_MAP_H
#define _OCC_MAP_H

#include "raycast.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>
#include <ros/ros.h>

using std::cout;
using std::endl;
using std::max;
using std::min;
using std::shared_ptr;
using std::vector;

namespace env
{
  class OccMap
  {
  public:
    OccMap() {}
    ~OccMap(){};
    void init(const ros::NodeHandle &nh);

    bool mapValid() { return is_global_map_valid_; }
    double getResolution() { return resolution_; }
    Eigen::Vector3d getOrigin() { return origin_; }
    Eigen::Vector3d getMapSize() { return map_size_; };
    bool isInMap(const Eigen::Vector3d &pos) const;

    // return whether this position is free in occ_map
    bool isStateValid(const Eigen::Vector3d &pos) const
    {
      Eigen::Vector3i idx = posToIndex(pos);
      if (!isInMap(idx))
        return false;
      return (occupancy_buffer_[idxToAddress(idx)] == false);
    };

    // check each grid in vector (p0, p1) isStateValid()
    // check first occupied grid, return false
    // all grids free, return true 
    // corrected(didn't check p1's validity, so conrner case:  p1 is first collision point)
    bool isSegmentValid(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, double max_dist = DBL_MAX) const
    {
      Eigen::Vector3d dp = p1 - p0;
      double dist = dp.norm();
      if (dist > max_dist)
      {
        return false;
      }
      RayCaster raycaster;
      bool need_ray = raycaster.setInput(p0 / resolution_, p1 / resolution_); //(ray start, ray end) pos2index
      if (!need_ray)     //start and end int the same occ_grid
        return true;
      Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
      Eigen::Vector3d ray_pt;
      if (!raycaster.step(ray_pt)) // skip the ray start point
        return true;
      while (raycaster.step(ray_pt))
      {
        Eigen::Vector3d tmp = (ray_pt + half) * resolution_;
        if (!this->isStateValid(tmp))
        {
          return false;
        }
      }
      Eigen::Vector3d tmp = (ray_pt + half) * resolution_;
      if (!this->isStateValid(tmp))
      {
        return false;
      }
      return true;
    }

    // new function, return bool and store first occ_grid pos in x_bounce
    // call this when sample new point and check segment, find bounce point(free) before first collision
    // add conrner case check:  p1 validity check
    // corner case2: 2 grid(1 is free, 2 is occ, return false, x_bounce is start)
    bool isSegmentValid_B(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, Eigen::Vector3d & x_bounce) const
    {
      RayCaster raycaster;
      bool need_ray = raycaster.setInput(p0 / resolution_, p1 / resolution_); // (ray start, ray end) pos2index
      if (!need_ray)     // start and end int the same occ_grid
        return true;
      Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
      Eigen::Vector3d ray_pt;
      if (!raycaster.step(ray_pt)) // skip the ray start point; couldn't step at first
        return true;
      Eigen::Vector3d last_free_point = (ray_pt + half) * resolution_;
      while (raycaster.step(ray_pt))
      {
        Eigen::Vector3d tmp = (ray_pt + half) * resolution_;
        if (!this->isStateValid(tmp))
        { 
          x_bounce = last_free_point;
          return false;
        }
        last_free_point = tmp;
      }
      // for corner case, check end point state
      Eigen::Vector3d tmp = (ray_pt + half) * resolution_;
      if (!this->isStateValid(tmp))
      { 
        x_bounce = last_free_point;
        return false;
      }
      return true;
    }

    // dir: (free_avgCenter - occ_avgCenter).normalized()
    // suitable for only two parts: occ and free
    // may bug when occ-free-occ or free-occ-free
    bool calNormVector(const Eigen::Vector3d &pos, Eigen::Vector3d &normVec) const
    {
      Eigen::Vector3i center_idx = posToIndex(pos);
      Eigen::Vector3d free_center(0,0,0), occ_center(0,0,0);
      int free_cnt = 0, occ_cnt = 0;

      for(int id_x = -2; id_x <= 2; id_x++)
        for(int id_y = -2; id_y <= 2; id_y++)
          for(int id_z = -2; id_z <= 2; id_z++)
          {
            if(id_x == 0 && id_y == 0 && id_z == 0)    // skip center
              continue;
            
            Eigen::Vector3i nei_idx, bias(id_x, id_y, id_z);
            nei_idx = center_idx + bias;

            if(!isInMap(nei_idx))
              continue;

            Eigen::Vector3d tmp;
            indexToPos(nei_idx, tmp);             // turn idx to pos(tmp)
            if(occupancy_buffer_[idxToAddress(nei_idx)]) // occ
            {
              occ_cnt++;
              occ_center += tmp;
            }else{                                // free
              free_cnt++;
              free_center += tmp;
            }
          }
      
      if(occ_cnt == 0 || free_cnt == 0)
      {
        ROS_WARN("[OCC_map] occ_cnt == 0 or free_cnt == 0, can not calculate norm vector.");
        return false;
      }

      normVec = (free_center / free_cnt - occ_center / occ_cnt).normalized();
      return true;
    }

    typedef shared_ptr<OccMap> Ptr;

  private:
    std::vector<bool> occupancy_buffer_;

    // map property
    Eigen::Vector3i grid_size_; // map size in index
    int grid_size_y_multiply_z_;

    int idxToAddress(const int &x_id, const int &y_id, const int &z_id) const;
    int idxToAddress(const Eigen::Vector3i &id) const;
    Eigen::Vector3i posToIndex(const Eigen::Vector3d &pos) const;
    void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id) const;
    void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos) const;
    void indexToPos(const int &x, const int &y, const int &z, Eigen::Vector3d &pos) const;
    bool isInMap(const Eigen::Vector3i &id) const;

    Eigen::Vector3d origin_, map_size_, min_range_, max_range_;
    double resolution_, resolution_inv_;

    // ros
    ros::NodeHandle node_;
    ros::Subscriber global_cloud_sub_;
    ros::Timer global_occ_vis_timer_;
    ros::Publisher glb_occ_pub_;

    void setOccupancy(const Eigen::Vector3d &pos);
    void globalOccVisCallback(const ros::TimerEvent &e);
    void globalCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr glb_cloud_ptr_;
    bool is_global_map_valid_;
  };

  inline int OccMap::idxToAddress(const int &x_id, const int &y_id, const int &z_id) const
  {
    return x_id * grid_size_y_multiply_z_ + y_id * grid_size_(2) + z_id;
  }

  inline int OccMap::idxToAddress(const Eigen::Vector3i &id) const
  {
    return id(0) * grid_size_y_multiply_z_ + id(1) * grid_size_(2) + id(2);
  }

  /* inline */ bool OccMap::isInMap(const Eigen::Vector3d &pos) const
  {
    Eigen::Vector3i idx;
    posToIndex(pos, idx);
    return isInMap(idx);
  }

  // 用位运算 或 写的，咋说呢，不明觉厉
  inline bool OccMap::isInMap(const Eigen::Vector3i &id) const
  {
    return ((id[0] | (grid_size_[0] - 1 - id[0]) | id[1] | (grid_size_[1] - 1 - id[1]) | id[2] | (grid_size_[2] - 1 - id[2])) >= 0);
  };

  inline void OccMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id) const
  {
    id(0) = floor((pos(0) - origin_(0)) * resolution_inv_);
    id(1) = floor((pos(1) - origin_(1)) * resolution_inv_);
    id(2) = floor((pos(2) - origin_(2)) * resolution_inv_);
  }

  // all index >= 0 min_range的相对索引
  inline Eigen::Vector3i OccMap::posToIndex(const Eigen::Vector3d &pos) const
  {
    Eigen::Vector3i id;
    id(0) = floor((pos(0) - origin_(0)) * resolution_inv_);
    id(1) = floor((pos(1) - origin_(1)) * resolution_inv_);
    id(2) = floor((pos(2) - origin_(2)) * resolution_inv_);
    return id;
  }

  inline void OccMap::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos) const
  {
    pos = origin_;
    pos(0) += (id(0) + 0.5) * resolution_;
    pos(1) += (id(1) + 0.5) * resolution_;
    pos(2) += (id(2) + 0.5) * resolution_;
  }

  inline void OccMap::indexToPos(const int &x, const int &y, const int &z, Eigen::Vector3d &pos) const
  {
    pos = origin_;
    pos(0) += (x + 0.5) * resolution_;
    pos(1) += (y + 0.5) * resolution_;
    pos(2) += (z + 0.5) * resolution_;
  }

} // namespace env

#endif
