# include <iostream>
# include <ros/ros.h>
# include <pcl_ros/point_cloud.h>
# include <pcl/point_types.h>
# include <pcl_conversions/pcl_conversions.h>
# include <pcl_ros/io/pcd_io.h>
# include <pcl/registration/icp.h>
# include <pcl/visualization/cloud_viewer.h>
# include <pcl/visualization/pcl_visualizer.h>
# include <pcl/console/time.h>
# include <pcl/filters/voxel_grid.h>
# include <pcl/filters/passthrough.h>
# include <pcl/common/transforms.h>
# include <sensor_msgs/PointCloud2.h>
# include <geometry_msgs/PointStamped.h>
# include <Eigen/Dense>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool first_fix = true;
bool first_cloud = true;

geometry_msgs::Point translation;

// The point clouds we will be using
PointCloudT::Ptr cloud_map (new PointCloudT);  // map point cloud
PointCloudT::Ptr cloud_origin (new PointCloudT);  // origin lidar point cloud
PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP lidar point cloud
PointCloudT::Ptr cloud_optimal (new PointCloudT);  // transformed point cloud by final transformation matrix
PointCloudT::Ptr cloud_filtered_map (new PointCloudT);  // filtered map point cloud
PointCloudT::Ptr cloud_filtered_car (new PointCloudT);  // filtered car point cloud

void print4x4Matrix (const Eigen::Matrix4d & matrix){
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

// callback function. Get the first gps data.
void cb_gps(const geometry_msgs::PointStamped::ConstPtr& msg){
    if (first_fix){
        translation = msg -> point;
        first_fix = false;
    }
}

// callback function. Get the first lidar points data.
void cb_cloud(const sensor_msgs::PointCloud2::ConstPtr& msg){
    if (first_cloud){
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud_origin);
        first_cloud = false;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ICP_node");

    ros::NodeHandle n;
    ros::Subscriber gps_sub = n.subscribe("fix", 1000, cb_gps);
    ros::Subscriber cloud_sub = n.subscribe("lidar_points", 1000, cb_cloud);
    ros::Rate rate(1);

    /////////////////////// parameters here ///////////////////////
    int iterations = 150;
    double divisor = 12;
    double theta = M_PI / divisor;
    double score = DBL_MAX;
    bool match = false;
    Eigen::Matrix4d optimal_transformation_matrix = Eigen::Matrix4d::Identity();
    /////////////////////// parameters here ///////////////////////

    while(ros::ok()){
        if (!first_fix && !first_cloud && !match){
            pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/steven/catkin_ws/src/loam_hw/src/data/map.pcd", *cloud_map); // read the pcd file

	    // downsampling for map, to remove unnecessary points.
	    pcl::PassThrough<PointT> pass_map;
	    pass_map.setInputCloud(cloud_map);
	    pass_map.setFilterFieldName("z");
	    pass_map.setFilterLimits(-2.0, 1.0);
	    pass_map.filter(*cloud_map);

	    // downsampling for car, to remove unnecessary points.
	    pcl::PassThrough<PointT> pass_car;
	    pass_car.setInputCloud(cloud_origin);
	    pass_car.setFilterFieldName("z");
	    pass_car.setFilterLimits(-2.0, 1.0);
	    pass_car.filter(*cloud_origin);

	    // keep rotating an angle theta to make different initial guesses for ICP algorithm
            for (int i = 0; i < divisor * 2; i++){
                std::cout << "\nThe " << i << " iterations\n";
                std::cout << "angle is " << theta * 180 / M_PI * i << std::endl;

                Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
		// rotation
                transformation_matrix(0, 0) = cos(theta * i);
                transformation_matrix(0, 1) = -sin(theta * i);
                transformation_matrix(1, 0) = sin(theta * i);
                transformation_matrix(1, 1) = cos(theta * i);
		// translation from body frame to world frame
                transformation_matrix(0, 3) = translation.x;
                transformation_matrix(1, 3) = translation.y;
                transformation_matrix(2, 3) = translation.z;
                pcl::transformPointCloud (*cloud_origin, *cloud_icp, transformation_matrix);

		// ICP algorithm
                pcl::IterativeClosestPoint<PointT, PointT> icp;
                icp.setMaximumIterations (iterations);
                icp.setInputSource (cloud_icp);
                icp.setInputTarget (cloud_map);
                icp.align (*cloud_icp);

                if (icp.hasConverged ()){
                    std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;

		    // score means distance. So the smaller the score, the better the performance
                    if (icp.getFitnessScore() < score){
                        score = icp.getFitnessScore();
			optimal_transformation_matrix = icp.getFinalTransformation().cast<double>();
                        optimal_transformation_matrix *= transformation_matrix;
                    }
                }
                else{
                    PCL_ERROR ("\nICP has not converged.\n");
                    return (-1);
                }
            }

            std::cout << "\noptimal score: " << score << std::endl;
            print4x4Matrix(optimal_transformation_matrix);
            pcl::transformPointCloud(*cloud_origin, *cloud_optimal, optimal_transformation_matrix);

	    // downsampling for map.
            pcl::VoxelGrid<PointT> map_filter;
            map_filter.setInputCloud(cloud_map);
            map_filter.setLeafSize(1, 1, 1);
            map_filter.filter(*cloud_filtered_map);

	    // downsampling for car.
            pcl::VoxelGrid<PointT> car_filter;
            map_filter.setInputCloud(cloud_optimal);
            map_filter.setLeafSize(1, 1, 1);
            map_filter.filter(*cloud_filtered_car);

            match = true;

	    // Visualization
  	    pcl::visualization::PCLVisualizer viewer ("ICP demo");
  	    // Create two vertically separated viewports
  	    int v1 (1);
  	    viewer.createViewPort (0.0, 0.0, 1.0, 1.0, v1);

  	    // The color we will be using
  	    float bckgr_gray_level = 0.0;  // Black
  	    float txt_gray_lvl = 1.0 - bckgr_gray_level;

  	    // Original point cloud is white
  	    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_map_color_h (cloud_filtered_map, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
  	                                                                               (int) 255 * txt_gray_lvl);
  	    viewer.addPointCloud (cloud_filtered_map, cloud_map_color_h, "cloud_map_v1", v1);

  	    // ICP aligned point cloud is red
  	    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_car_color_h (cloud_filtered_car, 180, 20, 20);
  	    viewer.addPointCloud (cloud_filtered_car, cloud_car_color_h, "cloud_icp_v1", v1);

  	    // Adding text descriptions in each viewport
  	    viewer.addText ("White: map point cloud\nRed: ICP aligned car point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);

  	    // Set background color
  	    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);

  	    // Set camera position and orientation
  	    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  	    viewer.setSize (1280, 1024);  // Visualiser window size

  	    // Display the visualiser
  	    while (!viewer.wasStopped ()) {
  	        viewer.spinOnce ();
  	    }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

