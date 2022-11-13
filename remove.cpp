#include <iostream>    
#include <pcl/ModelCoefficients.h>  
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/time.h>
#include <vtkPolyLine.h> // VTK用于可视化连接线

using namespace std;

//---------------------------------------------------邻接线条可视化------------------------------------------------
void addSupervoxelConnectionsToViewer(pcl::PointXYZRGBA& supervoxel_center, pcl::PointCloud<pcl::PointXYZRGBA>& adjacent_supervoxel_centers,
	std::string supervoxel_name, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();

	//遍历所有相邻点，并在相邻点对中添加一个中心点
	for (auto adjacent_itr = adjacent_supervoxel_centers.begin(); adjacent_itr != adjacent_supervoxel_centers.end(); ++adjacent_itr)
	{
		points->InsertNextPoint(supervoxel_center.data);
		points->InsertNextPoint(adjacent_itr->data);
	}
	// 创建一个polydata来存储所有内容
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	// Add the points to the dataset
	polyData->SetPoints(points);
	polyLine->GetPointIds()->SetNumberOfIds(points->GetNumberOfPoints());
	for (unsigned int i = 0; i < points->GetNumberOfPoints(); i++)
		polyLine->GetPointIds()->SetId(i, i);
	cells->InsertNextCell(polyLine);
	// 将这些线添加到数据集
	polyData->SetLines(cells);
	viewer->addModelFromPolyData(polyData, supervoxel_name);
}

int main(int argc, char** argv)
{
	pcl::console::TicToc time;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	// 读入点云PCD文件
	reader.read("E:\\PCL code\\X\\background_remove\\table.pcd", *cloud);
	cout << "Point cloud data: " << cloud->points.size() << " points" << endl;
	//创建分割时所需要的模型系数对象coefficients及存储内点的点索引集合对象inliers。
	cout << "算法计时开始" << endl;

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// 创建分割对象
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// 可选择配置，设置模型系数需要优化
	seg.setOptimizeCoefficients(true);
	// 必须配置，设置分割的模型类型、所用随机参数估计方法
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.15);// 距离阈值 单位m。距离阈值决定了点被认为是局内点时必须满足的条件
	//距离阈值表示点到估计模型的距离最大值。
	seg.setInputCloud(cloud);//输入点云
	seg.segment(*inliers, *coefficients);//实现分割，并存储分割结果到点集合inliers及存储平面模型系数coefficients
	//***********************************************************************
	// 去除地面
	time.tic();
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloud_filtered);

	cout << "平面提取时间:"<< time.toc() / 1000 << "s" << endl;

	cout << "Object cloud after filtering:: " << cloud_filtered->points.size() << " points" << endl;


//------------------------------------------------构建超体素---------------------------------------------------
	float voxel_resultion = 0.01f;   // 设置体素大小，该设置决定底层八叉树的叶子尺寸
	float seed_resultion = 0.1f;     // 设置种子大小，该设置决定超体素的大小

	pcl::SupervoxelClustering<pcl::PointXYZ> super(voxel_resultion, seed_resultion);
	super.setInputCloud(cloud_filtered);      // 输入点云
	super.setNormalImportance(5);    // 设置法向量的权重，即表面法向量影响超体素分割结果的比重。
	super.setColorImportance(0);     // 设置颜色在距离测试公式中的权重，即颜色影响超体素分割结果的比重。
	super.setSpatialImportance(0.4); // 设置空间距离在距离测试公式中的权重，较高的值会构建非常规则的超体素，较低的值产生的体素会按照法线
	std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZ>::Ptr >supervoxl_clustering;
	super.extract(supervoxl_clustering);
	cout << "超体素分割的体素个数为：" << supervoxl_clustering.size() << endl;
	// 获取点云对应的超体素分割标签
	pcl::PointCloud<pcl::PointXYZL>::Ptr supervoxel_cloud = super.getLabeledCloud();
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("VCCS"));
	int v3(0);
	viewer->createViewPort(0.6, 0, 0.9, 1,v3);
	viewer->addPointCloud(supervoxel_cloud, "supervoxel_cloud",v3);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "supervoxel_cloud",v3);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "supervoxel_cloud",v3);


	//左边窗口显示输入的点云,右边的窗口显示分割后的点云
	int v1(0) , v2(0);
	viewer->createViewPort(0, 0, 0.3, 1, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->createViewPort(0.3, 0, 0.6, 1, v2);
	viewer->setBackgroundColor(0, 0, 0, v2);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_in(cloud_filtered, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, color_in, "cloud_filtered", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_filtered", v2);

	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "cloud", v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud", v1);


	//-----------------------------------------获得体素点云的邻接单元----------------------------------------------
	multimap<uint32_t, uint32_t>SupervoxelAdjacency;
	super.getSupervoxelAdjacency(SupervoxelAdjacency);

	for (auto label_itr = SupervoxelAdjacency.cbegin(); label_itr != SupervoxelAdjacency.cend();)
	{
		uint32_t super_label = label_itr->first;//获取体素单元的标签
		pcl::Supervoxel<pcl::PointXYZ>::Ptr super_cloud = supervoxl_clustering.at(super_label);//把对应标签内的点云、体素质心、以及质心对应的法向量提取出来

		pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
		for (auto adjacent_itr = SupervoxelAdjacency.equal_range(super_label).first; adjacent_itr != SupervoxelAdjacency.equal_range(super_label).second; ++adjacent_itr)
		{
			pcl::Supervoxel<pcl::PointXYZ>::Ptr neighbor_supervoxel = supervoxl_clustering.at(adjacent_itr->second);
			adjacent_supervoxel_centers.push_back(neighbor_supervoxel->centroid_);
		}
		std::stringstream ss;
		ss << "supervoxel_" << super_label;
		addSupervoxelConnectionsToViewer(super_cloud->centroid_, adjacent_supervoxel_centers, ss.str(), viewer);
		label_itr = SupervoxelAdjacency.upper_bound(super_label);
	}
	// 等待直到可视化窗口关闭
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}

	return 0;

}

