#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

int main ()
{
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/ssd/dev/PointCloud/data-master/tutorials/room_scan1.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

//Pour avoir la valeur x,y,z de chaque point
//   for (const auto& point: *cloud)
//     std::cout << "    " << point.x
//               << " "    << point.y
//               << " "    << point.z << std::endl;

const double min_angle = 55.0; // Minimum angle in degrees
const double max_angle = 125.0;  // Maximum angle in degrees

pcl::PointIndices::Ptr indices(new pcl::PointIndices);

for (std::size_t i = 0; i < cloud->size(); ++i) {
    const double x = cloud->points[i].x;
    const double y = cloud->points[i].y;
    const double z = cloud->points[i].z;

    // Calculate the angle in degrees between the point and the x-axis
    const double angle = atan2(z, y) * 180.0 / M_PI;

    // on verifie pour les deux côtés
    if ((angle >= min_angle && angle <= max_angle)||(angle <= -min_angle && angle >= -max_angle)) {
        indices->indices.push_back(i); // Add the point's index to the filtered indices
    }
}

pcl::ExtractIndices<pcl::PointXYZ> extract;
extract.setInputCloud(cloud);
extract.setIndices(indices);
extract.setNegative(false); // Keep the filtered points
pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
extract.filter(*filtered_cloud);


  pcl::io::savePCDFileASCII ("/ssd/dev/PointCloud/data/filter1.pcd", *filtered_cloud);
  std::cerr << "Saved " << filtered_cloud->size () << " data points to test_pcd.pcd." << std::endl;

  for (const auto& point: *filtered_cloud)
    std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;
  return (0);
}