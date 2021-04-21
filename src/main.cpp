#include <iostream>
#include <pcl/pcl_config.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>


#include <tuple>
#include <chrono>
#include <thread>
#include <typeinfo>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include "lidar_obstacle_detection/pclarray.h"
#include "lidar_obstacle_detection/GDXBoxMessage.h"
#include "lidar_obstacle_detection/GDXBoxesMessage.h"
#include "lidar_obstacle_detection/TunningParam.h"



#include <pcl_ros/transforms.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

#define _GLIBCXX_USE_NANOSLEEP  

class myImages {
  public:
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    std::vector<boost::filesystem::path> stream;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::visualization::PCLVisualizer::Ptr viewer_processing;

};


struct Color
{

	float r, g, b;

	Color(float setR, float setG, float setB)
		: r(setR), g(setG), b(setB)
	{}
};

struct Box_type
{
	double xMin;
	double yMin;
	double zMin;
	double xMax;
	double yMax;
	double zMax;
};


Box_type BoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    pcl::PointXYZ minPoint, maxPoint;
    /*Get min and max coordinates in the cluster*/
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box_type box;
    box.xMin = minPoint.x;
    box.yMin = minPoint.y;
    box.zMin = minPoint.z;
    box.xMax = maxPoint.x;
    box.yMax = maxPoint.y;
    box.zMax = maxPoint.z;


	return box;
}

lidar_obstacle_detection::GDXBoxMessage BoundingBox_new(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    pcl::PointXYZ minPoint, maxPoint;
    /*Get min and max coordinates in the cluster*/
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    lidar_obstacle_detection::GDXBoxMessage box;
    box.xMin = minPoint.x;
    box.yMin = minPoint.y;
    box.zMin = minPoint.z;
    box.xMax = maxPoint.x;
    box.yMax = maxPoint.y;
    box.zMax = maxPoint.z;
    
    assert(minPoint.x < maxPoint.x);
    assert(minPoint.y < maxPoint.y);
    assert(minPoint.z < maxPoint.z);


	return box;
}

void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box_type box, int id, Color color, float opacity)
{
    // Limit opacity range between 0.0 and 1.0
	if(opacity > 1.0){
		opacity = 1.0;
        }
	if(opacity < 0.0){
		opacity = 0.0;
        }
	
    // Render a bounding box
	std::string cube = "box"+std::to_string(id);
    //viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube);
    viewer->addCube(box.xMin, box.xMax, box.yMin, box.yMax, box.zMin, box.zMax, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube); 
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);
    
    std::string cubeFill = "boxFill"+std::to_string(id);
    //viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFill);
    viewer->addCube(box.xMin, box.xMax, box.yMin, box.yMax, box.zMin, box.zMax, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill); 
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
}



// Function to render objects in the viewer

void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color)
{

	if(color.r==-1)
	{
		// Select color based off of cloud intensity
		pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> intensity_distribution(cloud,"intensity");
  		viewer->addPointCloud<pcl::PointXYZ>(cloud, intensity_distribution, name);
	}
	else
	{
		// Select color based off input value
		viewer->addPointCloud<pcl::PointXYZ> (cloud, name);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
	}

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}

// Default viewing angle options
enum CameraAngle
{
	XY, BEV, Side, FP
};

// Starting position of camera
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case BEV : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FP : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FP)
        viewer->addCoordinateSystem (1.0);
}

// Function to downsample the cloud based on Voxel Grid Filter
pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float leaf_size){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    voxel_grid_filter.setInputCloud(cloud);

    voxel_grid_filter.setLeafSize (leaf_size, leaf_size, leaf_size);
    voxel_grid_filter.filter(*cloud_filtered); 

    return cloud_filtered;
}

// Crops the entire point cloud to relevant area
pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint){

    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(cloud);
    

    crop_box.setMin(minPoint);
    crop_box.setMax(maxPoint);

    crop_box.filter(*cloud);    

    return cloud;
}

// Remove roof points since they are not relevant
pcl::PointCloud<pcl::PointXYZ>::Ptr remove_roof(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint){

    std::vector<int> indices;

    pcl::CropBox<pcl::PointXYZ> roof(true);

    roof.setMin(minPoint);
    roof.setMax(maxPoint);
    roof.setInputCloud(cloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for(int point:indices){
    	inliers->indices.push_back(point);
    }


    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);

    return cloud;
}



// Segment cloud into road and obstacles
// The biggest planar surface would be road and RANSAC is used to segment out this part

std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    // Declare output clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr road_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Declare vars for segmentation
    pcl::PointIndices::Ptr inliers_seg{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::SACSegmentation<pcl::PointXYZ> segmenter;

    // Set parameters
    segmenter.setOptimizeCoefficients(true);
    segmenter.setModelType(pcl::SACMODEL_PLANE);
    segmenter.setMethodType(pcl::SAC_RANSAC);
    segmenter.setDistanceThreshold(0.15);
    // segmenter.setMaxIterations(100);

    // set input cloud and output indices and coeff
    segmenter.setInputCloud(cloud);
    segmenter.segment (*inliers_seg, *coefficients);

	if (inliers_seg->indices.size () == 0)
	{
	  std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	}

    // Extract road point cloud using indices

	pcl::ExtractIndices<pcl::PointXYZ> extract_seg;
	extract_seg.setInputCloud(cloud);
	extract_seg.setIndices(inliers_seg);
    
	extract_seg.setNegative(false);
	extract_seg.filter(*road_cloud);


    extract_seg.setNegative(true);
    extract_seg.filter(*obstacle_cloud);

    // Return tuple of segmented road cloud and obstacle cloud
    return std::make_tuple(road_cloud, obstacle_cloud);

}

// Create clusters based on distance of points. 
// KD Tree based on euclidean distance is used to cluster points into cluster of obstacles
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> create_clusters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    // Array to store individual clusters
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    sensor_msgs::PointCloud2 rosCloud;


    // Initialize KD Tree with cloud
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree (new pcl::search::KdTree<pcl::PointXYZ>);
    kd_tree->setInputCloud(cloud);

    // Declare variables for clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    // Set parameters for clustering
    ec.setClusterTolerance(0.75);
	ec.setMinClusterSize(25);
	ec.setMaxClusterSize(2000);
	ec.setSearchMethod(kd_tree);
	ec.setInputCloud(cloud); 

    // Extract clusters   
    ec.extract(cluster_indices);

    // Append individual clusters to an array
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            cloud_cluster->points.push_back(cloud->points[*pit]);
        }

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // std::cout<< "Cluster with " << cloud_cluster->points.size() << " points" << std::endl;
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud (new pcl::PointCloud<pcl::PointXYZRGB>); // creates a shared pointer
        // pcl::toROSMsg(*cloud_cluster, rosCloud);

        clusters.push_back(cloud_cluster);
    
    }

    return clusters;
}


// Draw bounding boxes around clusters to indicate obstacles
void render_all_boxes(pcl::visualization::PCLVisualizer::Ptr& viewer, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters){

    int clusterId = 0;

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : clusters)
        {

        renderPointCloud(viewer, cluster,"obstCloud"+std::to_string(clusterId),Color(1,1,0));

        Box_type box = BoundingBox(cluster);
        renderBox(viewer,box,clusterId,Color(0.9,0,0), 0.5f);

        ++clusterId;
        }

    
}

// Obtain all boxes 

lidar_obstacle_detection::GDXBoxesMessage obtain_all_boxes(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters){

    lidar_obstacle_detection::GDXBoxesMessage boxes;


    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : clusters)
        {
            lidar_obstacle_detection::GDXBoxMessage box = BoundingBox_new(cluster);
            boxes.boxes.push_back(box);
        }
    return boxes;
    
}

// Returns paths of all .pcd files in a directory

std::vector<boost::filesystem::path> streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

// Load point clouds from path

pcl::PointCloud<pcl::PointXYZ>::Ptr loadPcd(std::string file)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }

    return cloud;
}

void TunerCallback(const lidar_obstacle_detection::TunningParam::ConstPtr& msg) {
    std::cout << msg->tunningParam1 << std::endl;
}

// void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, std::vector<boost::filesystem::path>& stream, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::visualization::PCLVisualizer::Ptr& viewer_processing) {
void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // auto streamIterator = this->stream.begin();
    // std::cout << "streamIterator:::"<< (*streamIterator).string() << std::endl;

    // this->viewer_processing->removeAllPointClouds();
    // this->viewer_processing->removeAllShapes();

    // this->cloud = loadPcd((*streamIterator).string());
    ros::Rate loop_rate(10);
    ros::NodeHandle n;
    ros::Publisher pc_pub = n.advertise<PointCloud>("/downsampled_cloud_road", 1);
    int threshold = 1000;
    while (ros::ok())
    {

        pcl::PCLPointCloud2 pcl_pc2;
        // ROS_INFO("point_cloud_size: %ld" , );
        if (msg->data.size() << threshold){
            
        }
        else{
            pcl_conversions::toPCL(*msg,pcl_pc2);
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = temp_cloud;

            // Output cloud for downsampled cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

            // Downsample the point cloud to lower memory usage and faster processing
            cloud_filtered = downsample_cloud(cloud,0.15f);

            // Crop point cloud to relevant area
            cloud_filtered = crop_cloud(cloud_filtered,Eigen::Vector4f(-21, -7, -3, 1),Eigen::Vector4f( 31, 8, 6, 1));

            // Remove car roof points 
            cloud_filtered = remove_roof(cloud_filtered,Eigen::Vector4f(-1.4,-1.6,-1,1), Eigen::Vector4f(2.5,1.6,-0.4,1));

            // Segment obstacles and road
            std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmented_clouds;
            segmented_clouds = segment_clouds(cloud_filtered);

            // Store segmented point clouds in individual PointXYZ cloud
            PointCloud::Ptr road_cloud = std::get<0>(segmented_clouds);
            PointCloud::Ptr obstacle_cloud = std::get<1>(segmented_clouds);
            
            //////////////////////// test

            // std::cout << "cloud_size:::" <<  road_cloud->points.size()<< std::endl;
            // for (unsigned int i=0; i < road_cloud->points.size() ; i=i+1) {
            //     std::cout << "pointx::"<< road_cloud->points[i].x << std::endl;
            // }

            pc_pub.publish(*road_cloud);

            ros::Publisher pc_pub_2 = n.advertise<PointCloud>("/downsampled_cloud_obstacle", 1);
            pc_pub_2.publish(*obstacle_cloud);
            // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
            // pc_pub.publish(*road_cloud);


            ////////////////////////////// 

            // // Render road (segmented from original point cloud) in the viewer
            // renderPointCloud(this->viewer_processing,road_cloud,"planeCloud",Color(1,1,1));

            // // Create clusters of obstacle from raw points
            // create_clusters(obstacle_cloud);
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = create_clusters(obstacle_cloud);
            lidar_obstacle_detection::GDXBoxesMessage new_boxes = obtain_all_boxes(clusters);

            // lidar_obstacle_detection::pclarray pclarray;
            // pclarray.cloudarray = clusters;
            
            ros::Publisher pc_pub_3 = n.advertise<lidar_obstacle_detection::GDXBoxesMessage>("/boxes", 1);
            pc_pub_3.publish(new_boxes);

            lidar_obstacle_detection::GDXBoxMessage new_box = new_boxes.boxes[0];
            ros::Publisher pc_pub_4 = n.advertise<lidar_obstacle_detection::GDXBoxMessage>("/box", 1);
            pc_pub_4.publish(new_box);

            // // Render bounding box around obstacles
            // render_all_boxes(this->viewer_processing,clusters);

            // // Sleep for 40ms to slow down the viewer
            // std::this_thread::sleep_for(std::chrono::milliseconds(40));

            // // Go to the next point cloud
            // // streamIterator++;

            // // If stream reaches the last point cloud, then start again from the first point cloud
            // // if(streamIterator == this->stream.end())
            // //     streamIterator = this->stream.begin();
            // this->viewer_processing->spinOnce();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

}

int main(int argc, char **argv){

    myImages myimage_obj;

    // pcl::visualization::PCLVisualizer::Ptr viewer_processing (new pcl::visualization::PCLVisualizer ("LIDAR Obstacle Detection"));
    // myimage_obj.viewer_processing = viewer_processing;
    
    // // Sets Default camera angle to XY or Birds Eye View or Side view or First Person view
    // CameraAngle setAngle = FP;              // XY or BEV or Side or FP
    // initCamera(setAngle, myimage_obj.viewer_processing);
    
    // std::vector<boost::filesystem::path> stream = streamPcd("/home/chen1804/catkin_workspace/src/lidar_obstacle_detection/data");
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    // pcl::visualization::PCLVisualizer::Ptr viewer_processing (new pcl::visualization::PCLVisualizer ("LIDAR Obstacle Detection"));
    
    // // Sets Default camera angle to XY or Birds Eye View or Side view or First Person view
    // CameraAngle setAngle = FP;              // XY or BEV or Side or FP
    // initCamera(setAngle, viewer_processing);

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/os_cloud_node/points", 1, pointcloudCallback);
    ros::Subscriber sub2 = n.subscribe("/TunningParam", 1, TunerCallback);

    ros::Rate loop_rate(10);

    // , stream, cloud, viewer_processing
	while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
        
    return 0;


}
