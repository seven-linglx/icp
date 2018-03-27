#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/visualization/pcl_visualizer.h>

#define PI 3.1415926
using namespace pcl;
using namespace std;

void readtransformMatrix(Eigen::Matrix4f &lidar_transform)
{
//    float data_read[6]={-0.35, 0.0 , 1.92 , 41 , -5 ,0};
    double data_read[6]={0, 0 , 0 , 0 , 0 , 8};
    //readFromCalibration(data_read);
    float rx = data_read[0]*3.1415926/180;
    float ry = data_read[1]*3.1415926/180;
    float rz = data_read[2]*3.1415926/180;
    float tx = data_read[3];
    float ty = data_read[4];
    float tz = data_read[5];

    float cx=cos(rx);
    float sx=sin(rx);
    float cy=cos(ry);
    float sy=sin(ry);
    float cz=cos(rz);
    float sz=sin(rz);


    lidar_transform(0,0)=cy*cz;
    lidar_transform(0,1)=-cx*sz+sx*sy*cz;
    lidar_transform(0,2)=cx*sy*cz+sx*sz;
    lidar_transform(1,0)=cy*sz;
    lidar_transform(1,1)=cx*cz+sx*sy*sz;
    lidar_transform(1,2)=cx*sy*sz-sx*cz;
    lidar_transform(2,0)=-sy;
    lidar_transform(2,1)=sx*cy;
    lidar_transform(2,2)=cx*cy;
    lidar_transform(0,3)=tx;
    lidar_transform(1,3)=ty;
    lidar_transform(2,3)=tz;
    lidar_transform(3,3)=1;

}

Eigen::Matrix4f point_to_plane (pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud1,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2)
{
  console::TicToc t;
  t.tic();
  pcl::PointCloud<pcl::PointNormal>::Ptr src(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*cloud1, *src);
  pcl::PointCloud<pcl::PointNormal>::Ptr tgt(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*cloud2, *tgt);

  pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est;
  norm_est.setSearchMethod (pcl::search::KdTree<pcl::PointNormal>::Ptr (new pcl::search::KdTree<pcl::PointNormal>));
  norm_est.setKSearch (10);
  norm_est.setInputCloud (tgt);
  norm_est.compute (*tgt);

  pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
  typedef pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal> PointToPlane;
  boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
  icp.setTransformationEstimation(point_to_plane);     // key

  icp.setInputSource(src);
  icp.setInputTarget(tgt);
  //icp.setRANSACOutlierRejectionThreshold(ransac_par);
  icp.setRANSACIterations(20);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-3);
  pcl::PointCloud<pcl::PointNormal> output;
  icp.align(output);//align 的另一个重载可以设置一个初始矩阵guess
  t.toc_print();
  return icp.getFinalTransformation();
}

Eigen::Matrix4f point_to_point(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_source,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_target)
{
  console::TicToc t;
  t.tic();
	// ICP
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//pcl::IterativeClosestPointWithNormals<pcl::PointXYZ, pcl::PointXYZ> icp;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
	tree1->setInputCloud(cloud_source);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
	tree2->setInputCloud(cloud_target);
	icp.setSearchMethodSource(tree1);
	icp.setSearchMethodTarget(tree2);
	icp.setInputSource(cloud_source);
	icp.setInputTarget(cloud_target);
	icp.setMaxCorrespondenceDistance(150);//1500
	icp.setTransformationEpsilon(1e-3);
	icp.setEuclideanFitnessEpsilon(0.01);//0.1
	icp.setMaximumIterations(100);//100
  pcl::PointCloud<pcl::PointXYZ> output;
	icp.align(output);
	//std::cout << transformation << std::endl;
  t.toc_print();
	return icp.getFinalTransformation();
}

void displayAngel(Eigen::Matrix4f &transformation)
{
double rx,ry,rz,tx,ty,tz;
ry = (180/PI)*atan(-(transformation(2,0) / sqrt(pow(transformation(0,0), 2) + pow(transformation(1,0), 2))));
rz = (180/PI)*atan((transformation(1,0)/transformation(0,0)));
rx = (180/PI)*atan((transformation(2,1)/transformation(2,2)));
tx = transformation(0,3);
ty = transformation(1,3);
tz = transformation(2,3);
std::cout << rx <<'\n'<< ry <<'\n'<< rz <<'\n'<< tx <<'\n'<< ty <<'\n'<< tz << std::endl;
}

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_transform (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);

    std::string pcd_source_path,pcd_target_path;
    pcd_source_path ="/home/linglx/Data/calibration_for_four_lidar/old/43-1.pcd";
    pcd_target_path ="/home/linglx/Data/calibration_for_four_lidar/old/45-1.pcd";
	// load pcd file
	if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_source_path, *cloud_source)==-1) {
		std::cout << "load source failed!" << std::endl;
		return -1;
	}
	std::cout << "source loaded!" << std::endl;

	if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_target_path, *cloud_target)==-1) {
		std::cout << "load target failed!" << std::endl;
		return -1;
	}
	std::cout << "target loaded!" << std::endl;

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    readtransformMatrix(transform);
    pcl::transformPointCloud (*cloud_target, *cloud_target_transform, transform);

	Eigen::Matrix4f transformation = point_to_point(cloud_source, cloud_target);
    displayAngel(transformation);

    Eigen::Matrix4f transformation_point_to_plane = point_to_plane(cloud_source, cloud_target);
    pcl::transformPointCloud (*cloud_source, *Final, transformation_point_to_plane);

	// std::cout << transformation << std::endl;
	// display
    displayAngel(transformation_point_to_plane);
	pcl::visualization::PCLVisualizer p;
    p.setWindowName("Could after calibration");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_r_h(Final, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_after_transform_h (cloud_target_transform, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h (cloud_target, 0, 0, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h (cloud_source, 255, 255, 255);
	p.addPointCloud(Final, src_r_h,"source_r");
	//p.addPointCloud(cloud_target_transform, tgt_after_transform_h, "target_after_transform");
	p.addPointCloud(cloud_target, tgt_h, "target");
	p.addPointCloud(cloud_source, src_h, "source");
    p.setSize(1200,900);
    p.spin();
	return 0;
}
