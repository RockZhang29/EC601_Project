 #include <iostream>
 #include <pcl/io/pcd_io.h>
 #include <pcl/point_types.h>
 #include <pcl/filters/statistical_outlier_removal.h>
 
 int
 main ()
 {
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> ("object3d.pcd", *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (0.5);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("inliers.pcd", *cloud_filtered, false);

  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> ("outliers.pcd", *cloud_filtered, false);

  return (0);
}
