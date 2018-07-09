// pcl_demo.cpp : �������̨Ӧ�ó������ڵ㡣
//
#include <pcl/visualization/cloud_viewer.h>  
#include <iostream>//��׼C++���е�������������ͷ�ļ���  
#include <pcl/io/io.h>  
#include <pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���  
#include <pcl/io/ply_io.h>  
#include <pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ���  
int user_data;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 1.0);   //���ñ�����ɫ  
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
	pcl::visualization::CloudViewer viewer("Cloud Viewer");     //����viewer����  

	viewer.showCloud(cloud);

	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	system("pause");

	return 0;

}

