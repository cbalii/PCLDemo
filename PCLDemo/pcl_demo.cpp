// pcl_demo.cpp : 定义控制台应用程序的入口点。
//
#include <pcl/visualization/cloud_viewer.h>  
#include <iostream>//标准C++库中的输入输出类相关头文件。  
#include <pcl/io/io.h>  
#include <pcl/io/pcd_io.h>//pcd 读写类相关的头文件。  
#include <pcl/io/ply_io.h>  
#include <pcl/point_types.h> //PCL中支持的点类型头文件。  
int user_data;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 1.0);   //设置背景颜色  
}

int main()
{
	//cout << "size of short "<<sizeof(short) << endl;
	//cout << "size of PointT " << sizeof(pcl::PointXYZ) << endl;

	//unsigned char pMem[] = {0x71,0x33};
	//unsigned short *p = (unsigned short*)pMem;
	//int temp = 0x0225-0x0067;
	//printf("%d\n", temp);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::io::loadPCDFile("temp.pcd", *cloud);
	pcl::visualization::CloudViewer viewer("Cloud Viewer");     //创建viewer对象  

	viewer.showCloud(cloud);

	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	system("pause");

	return 0;

}

