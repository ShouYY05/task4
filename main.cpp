#include<iostream>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL)
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include<ctime>
#include<cstdlib>
//#include <windows.h>

using namespace pcl;
using namespace std;
typedef PointXYZ PoinT;

int *rand_rgb(){//随机产生颜色
	int *rgb = new int[3];	
	rgb[0] = rand() % 255;
	rgb[1] = rand() % 255;
	rgb[2] = rand() % 255;
	return rgb;
}
int main(){
	//点云的读取*********************************************************
	PointCloud<PoinT>::Ptr cloud(new PointCloud<PoinT>);
	if (io::loadPCDFile("/home/shouyiyang/桌面/视觉实习生任务/任务4/点云/1.pcd", *cloud) == -1)
	{
		PCL_ERROR("read false");
		return 0;
	}
    else cout << "open pcd file!" <<endl;
    //体素化下采样******************************************************
	VoxelGrid<PoinT> vox;
	PointCloud<PoinT>::Ptr vox_cloud(new PointCloud<PoinT>);
	vox.setInputCloud(cloud);
	vox.setLeafSize(0.01, 0.01, 0.01);
	vox.filter(*vox_cloud);
	//去除噪声点********************************************************
	StatisticalOutlierRemoval<PoinT>sor;
	PointCloud<PoinT>::Ptr sor_cloud(new PointCloud<PoinT>);
	sor.setMeanK(10);
	sor.setInputCloud(vox_cloud);
	sor.setStddevMulThresh(0.2);
	sor.filter(*sor_cloud);
	//平面分割(RANSAC)********************************************************
	SACSegmentation<PoinT> sac;
	PointIndices::Ptr inliner(new PointIndices);
	ModelCoefficients::Ptr coefficients(new ModelCoefficients);
	PointCloud<PoinT>::Ptr sac_cloud(new PointCloud<PoinT>);
	sac.setInputCloud(sor_cloud);
	sac.setMethodType(SAC_RANSAC);
	sac.setModelType(SACMODEL_PLANE);
	sac.setMaxIterations(100);
	sac.setDistanceThreshold(0.02);
	//提取平面(展示并输出)******************************************************
	PointCloud<PoinT>::Ptr ext_cloud(new PointCloud<PoinT>);
	PointCloud<PoinT>::Ptr ext_cloud_rest(new PointCloud<PoinT>);
	visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3d view"));

	int i = sor_cloud->size(), j = 0;
	ExtractIndices<PoinT>ext;
	srand((unsigned)time(NULL));//刷新时间的种子节点需要放在循环体外面
	while (sor_cloud->size()>i*0.3)//当提取的点数小于总数的3/10时，跳出循环
	{
		ext.setInputCloud(sor_cloud);
		sac.segment(*inliner, *coefficients);
		if (inliner->indices.size()==0)
		{
			break;
		}
		//按照索引提取点云*************
		ext.setIndices(inliner);
		ext.setNegative(false);
		ext.filter(*ext_cloud);
		ext.setNegative(true);
		ext.filter(*ext_cloud_rest);
		//*****************************
		*sor_cloud = *ext_cloud_rest;
		stringstream ss;
		ss <<"C:\\Users\\Administrator\\Desktop\\"<<"ext_plane_clouds" << j << ".pcd";//路径加文件名和后缀
		io::savePCDFileASCII(ss.str(), *ext_cloud);//提取的平面点云写出
		int *rgb = rand_rgb();//随机生成0-255的颜色值
		visualization::PointCloudColorHandlerCustom<PoinT>rgb1(ext_cloud,rgb[0],rgb[1],rgb[2]);//提取的平面不同彩色展示
		delete[]rgb;
		viewer->addPointCloud(ext_cloud, rgb1,ss.str());
		j++;
	}
	viewer->spinOnce(1000);
	//欧式聚类*******************************************************
	vector<PointIndices>ece_inlier;
	search::KdTree<PoinT>::Ptr tree(new search::KdTree<PoinT>);
	EuclideanClusterExtraction<PoinT> ece;
	ece.setInputCloud(sor_cloud);
	ece.setClusterTolerance(0.02);
	ece.setMinClusterSize(100);
	ece.setMaxClusterSize(20000);
	ece.setSearchMethod(tree);
	ece.extract(ece_inlier);
	//聚类结果展示***************************************************
	ext.setInputCloud(sor_cloud);
	visualization::PCLVisualizer::Ptr viewer2(new visualization::PCLVisualizer("Result of EuclideanCluster"));
	srand((unsigned)time(NULL));
	for (int i = 0; i < ece_inlier.size();i++)
	{
		PointCloud<PoinT>::Ptr cloud_copy(new PointCloud<PoinT>);
		vector<int> ece_inlier_ext = ece_inlier[i].indices;
		copyPointCloud(*sor_cloud, ece_inlier_ext, *cloud_copy);//按照索引提取点云数据
		stringstream ss1;
		ss1 <<"C:\\Users\\Administrator\\Desktop\\"<< "EuclideanCluster_clouds" << j<<".pcd";
		io::savePCDFileASCII(ss1.str(), *ext_cloud);//欧式聚类结果写出
		int *rgb1 = rand_rgb();
		visualization::PointCloudColorHandlerCustom<PoinT>rgb2(ext_cloud, rgb1[0], rgb1[1], rgb1[2]);
		delete[]rgb1;
		viewer2->addPointCloud(cloud_copy, rgb2,ss1.str());
		j++;
	}
	viewer2->spin();
	return 0;
}



/*
#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <boost/thread/thread.hpp>
 
using namespace pcl;
 
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("3D Viewer"));
 
int main()
{

    //定义点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 
    // 定义读取对象
    pcl::PCDReader reader;
    // 读取点云文件
    reader.read<pcl::PointXYZ> ("/home/shouyiyang/桌面/视觉实习生任务/任务4/点云/1.pcd",*cloud);
    //reader.read<pcl::PointXYZ> ("/home/shouyiyang/桌面/视觉实习生任务/任务4/点云/2.pcd",*cloud);
    //reader.read<pcl::PointXYZ> ("/home/shouyiyang/桌面/视觉实习生任务/任务4/点云/3.pcd",*cloud);
    //reader.read<pcl::PointXYZ> ("/home/shouyiyang/桌面/视觉实习生任务/任务4/点云/4.pcd",*cloud);
    //reader.read<pcl::PointXYZ> ("/home/shouyiyang/桌面/视觉实习生任务/任务4/点云/5.pcd",*cloud);

    while(true)
    {
 
        viewer1->removeAllPointClouds();  // 移除当前所有点云
        viewer1->addPointCloud(cloud, "test");  
        viewer1->updatePointCloud(cloud, "test"); 
        viewer1->spinOnce(0.0000000000001);
        

    }
 
}

*/