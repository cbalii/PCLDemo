#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/impl/bilateral.hpp>
PCL_INSTANTIATE(BilateralFilter, PCL_XYZ_POINT_TYPES);
template class PCL_EXPORTS pcl::BilateralFilter<pcl::PointXYZ>;
template class PCL_EXPORTS pcl::BilateralFilter<pcl::PointXYZI>;
template class PCL_EXPORTS pcl::BilateralFilter<pcl::PointXYZRGB>;
