# 基于PCL的RANSAC地面提取和超体素分割

## 摘要

（一）利用==RANSAC算法==将桌子的背景提取（即地面分割）

（二）对地面分割后的桌子进行==超体素分割==

## 基本原理

### RANSAC原理

从原始点云中随机选取几个点构成平面，当点云距离平面较近时加入这个平面，不断迭代，最终所得平面点的个数达到一定数量即平面拟合完成

注意参数：

==codfficients 模型参数==

==inliers 内点索引==

![image-20220308140344665](https://gitee.com/yao-tuo/typora/raw/master/img/image-20220308140344665.png)

### 超体素分割原理

 ==pcl::SupervoxelClustering==实现的方法是VCCS（Voxel Cloud Connectivity Segmentation）方法，它属于一种超像素方法，该方法可以产生3D点云数据的像素级分割，获得的分割结果被称为超体素。在处理物体边界方面，超体素要优于现有的基于二维图像的方法。同时，该方法保持较高的效率，利用空间==八叉树==结构，VCCS使用==k-均值聚类的区域增长==来直接对点云机型超体素分割。

  得到的超体素有两个重要特性：

第一，在3D空间内，均匀分布，该特性，通过在点云空间中均匀设定种子达到

第二，除非体素空间上是相连的，否则超体素不能跨越边界。利用八叉树结构，可以判断叶节点是否相邻。在体素化的3D空间内，超体素保持相邻关系，是指这些体素之间，分享共用的面、边和顶点

## 代码实现

### 调用的库

```cpp
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
```

### RANSAC部分

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PCDReader reader;
// 读入点云PCD文件
reader.read("E:\\PCL code\\X\\background_remove\\table.pcd", *cloud);
cout << "Point cloud data: " << cloud->points.size() << " points" << endl;

//创建分割时所需要的模型系数对象coefficients及存储内点的点索引集合对象inliers。
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
```

### 算法计时

```cpp
pcl::console::TicToc time;
time.tic();
cout << "平面提取时间:"<< time.toc() / 1000 << "s" << endl;
```

### 超体素分割

```cpp
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
```

### 可视化

```cpp
int v1(0) , v2(0);
viewer->createViewPort(0, 0, 0.3, 1, v1);
viewer->setBackgroundColor(0, 0, 0, v1);
viewer->createViewPort(0.3, 0, 0.6, 1, v2);
viewer->setBackgroundColor(0, 0, 0, v2);

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_in(cloud_filtered, 255, 0, 0);
viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, color_in, "cloud_filtered", v2);
viewer>setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_filtered", v2);

viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);
viewer>setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "cloud", v1);
viewer>setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud", v1);

// 获取点云对应的超体素分割标签
pcl::PointCloud<pcl::PointXYZL>::Ptr supervoxel_cloud = super.getLabeledCloud();
pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("VCCS"));
int v3(0);
viewer->createViewPort(0.6, 0, 0.9, 1,v3);
viewer->addPointCloud(supervoxel_cloud, "supervoxel_cloud",v3);
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "supervoxel_cloud",v3);
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "supervoxel_cloud",v3);
```



## 代码运行结果

![image-20221113160434167](https://image-1312312327.cos.ap-shanghai.myqcloud.com/image-20221113160434167.png)

![image-20221113160411277](https://image-1312312327.cos.ap-shanghai.myqcloud.com/image-20221113160411277.png)