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
#include <vtkPolyLine.h> // VTK���ڿ��ӻ�������

using namespace std;

//---------------------------------------------------�ڽ��������ӻ�------------------------------------------------
void addSupervoxelConnectionsToViewer(pcl::PointXYZRGBA& supervoxel_center, pcl::PointCloud<pcl::PointXYZRGBA>& adjacent_supervoxel_centers,
	std::string supervoxel_name, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New();

	//�����������ڵ㣬�������ڵ�������һ�����ĵ�
	for (auto adjacent_itr = adjacent_supervoxel_centers.begin(); adjacent_itr != adjacent_supervoxel_centers.end(); ++adjacent_itr)
	{
		points->InsertNextPoint(supervoxel_center.data);
		points->InsertNextPoint(adjacent_itr->data);
	}
	// ����һ��polydata���洢��������
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	// Add the points to the dataset
	polyData->SetPoints(points);
	polyLine->GetPointIds()->SetNumberOfIds(points->GetNumberOfPoints());
	for (unsigned int i = 0; i < points->GetNumberOfPoints(); i++)
		polyLine->GetPointIds()->SetId(i, i);
	cells->InsertNextCell(polyLine);
	// ����Щ����ӵ����ݼ�
	polyData->SetLines(cells);
	viewer->addModelFromPolyData(polyData, supervoxel_name);
}

int main(int argc, char** argv)
{
	pcl::console::TicToc time;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	// �������PCD�ļ�
	reader.read("E:\\PCL code\\X\\background_remove\\table.pcd", *cloud);
	cout << "Point cloud data: " << cloud->points.size() << " points" << endl;
	//�����ָ�ʱ����Ҫ��ģ��ϵ������coefficients���洢�ڵ�ĵ��������϶���inliers��
	cout << "�㷨��ʱ��ʼ" << endl;

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// �����ָ����
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// ��ѡ�����ã�����ģ��ϵ����Ҫ�Ż�
	seg.setOptimizeCoefficients(true);
	// �������ã����÷ָ��ģ�����͡���������������Ʒ���
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.15);// ������ֵ ��λm��������ֵ�����˵㱻��Ϊ�Ǿ��ڵ�ʱ�������������
	//������ֵ��ʾ�㵽����ģ�͵ľ������ֵ��
	seg.setInputCloud(cloud);//�������
	seg.segment(*inliers, *coefficients);//ʵ�ַָ���洢�ָ������㼯��inliers���洢ƽ��ģ��ϵ��coefficients
	//***********************************************************************
	// ȥ������
	time.tic();
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloud_filtered);

	cout << "ƽ����ȡʱ��:"<< time.toc() / 1000 << "s" << endl;

	cout << "Object cloud after filtering:: " << cloud_filtered->points.size() << " points" << endl;


//------------------------------------------------����������---------------------------------------------------
	float voxel_resultion = 0.01f;   // �������ش�С�������þ����ײ�˲�����Ҷ�ӳߴ�
	float seed_resultion = 0.1f;     // �������Ӵ�С�������þ��������صĴ�С

	pcl::SupervoxelClustering<pcl::PointXYZ> super(voxel_resultion, seed_resultion);
	super.setInputCloud(cloud_filtered);      // �������
	super.setNormalImportance(5);    // ���÷�������Ȩ�أ������淨����Ӱ�쳬���طָ����ı��ء�
	super.setColorImportance(0);     // ������ɫ�ھ�����Թ�ʽ�е�Ȩ�أ�����ɫӰ�쳬���طָ����ı��ء�
	super.setSpatialImportance(0.4); // ���ÿռ�����ھ�����Թ�ʽ�е�Ȩ�أ��ϸߵ�ֵ�ṹ���ǳ�����ĳ����أ��ϵ͵�ֵ���������ػᰴ�շ���
	std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZ>::Ptr >supervoxl_clustering;
	super.extract(supervoxl_clustering);
	cout << "�����طָ�����ظ���Ϊ��" << supervoxl_clustering.size() << endl;
	// ��ȡ���ƶ�Ӧ�ĳ����طָ��ǩ
	pcl::PointCloud<pcl::PointXYZL>::Ptr supervoxel_cloud = super.getLabeledCloud();
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("VCCS"));
	int v3(0);
	viewer->createViewPort(0.6, 0, 0.9, 1,v3);
	viewer->addPointCloud(supervoxel_cloud, "supervoxel_cloud",v3);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "supervoxel_cloud",v3);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "supervoxel_cloud",v3);


	//��ߴ�����ʾ����ĵ���,�ұߵĴ�����ʾ�ָ��ĵ���
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


	//-----------------------------------------������ص��Ƶ��ڽӵ�Ԫ----------------------------------------------
	multimap<uint32_t, uint32_t>SupervoxelAdjacency;
	super.getSupervoxelAdjacency(SupervoxelAdjacency);

	for (auto label_itr = SupervoxelAdjacency.cbegin(); label_itr != SupervoxelAdjacency.cend();)
	{
		uint32_t super_label = label_itr->first;//��ȡ���ص�Ԫ�ı�ǩ
		pcl::Supervoxel<pcl::PointXYZ>::Ptr super_cloud = supervoxl_clustering.at(super_label);//�Ѷ�Ӧ��ǩ�ڵĵ��ơ��������ġ��Լ����Ķ�Ӧ�ķ�������ȡ����

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
	// �ȴ�ֱ�����ӻ����ڹر�
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}

	return 0;

}

