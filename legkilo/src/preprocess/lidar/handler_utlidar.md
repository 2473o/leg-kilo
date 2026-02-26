# UTLIDAR Handler

From [point_lio_unilidar](https://github.com/unitreerobotics/point_lio_unilidar/blob/main/src/preprocess.h)

```cpp
/**
 * @brief Unilidar Point Type
 */
namespace unilidar_ros {
struct Point
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY
  std::uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}
POINT_CLOUD_REGISTER_POINT_STRUCT(unilidar_ros::Point,
  (float, x, x)(float, y, y)(float, z, z)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
  (float, time, time)
)

```

```cpp

void Preprocess::unilidar_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<unilidar_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;

    pl_surf.reserve(plsize);

    // std::cout << "plsize = " << plsize << ", given_offset_time = " << given_offset_time << std::endl;
    int countElimnated = 0;
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;

      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      
      added_pt.intensity = pl_orig.points[i].intensity;

      added_pt.curvature = pl_orig.points[i].time * time_unit_scale; 

      if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind * blind))
      {
        pl_surf.points.push_back(added_pt);
      }
      else
      {
        countElimnated++;
      }
    }

    // std::cout << "pl_surf.size() = " << pl_surf.size() << ", countElimnated = " << countElimnated << std::endl;
    
}
```