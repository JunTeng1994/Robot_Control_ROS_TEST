#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//read matrix by file name
bool ReadMatrix(std::string FileName, Eigen::Matrix4d& trans)
{
    cv::FileStorage fs(FileName, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        return false;
    }
    cv::Mat transMat;

    fs["TransMat"] >> transMat;
    fs.release();

    cv2eigen(transMat, trans);
    return true;
}

//save matrix in the file
bool SaveMatrix(std::string FileName, Eigen::Matrix4d transMat)
{
    cv::FileStorage fs(FileName, cv::FileStorage::WRITE);
    if (!fs.isOpened())
    {
        return false;
    }
    cv::Mat cvtransMat;
    eigen2cv(transMat, cvtransMat);
    fs << "TransMat" << cvtransMat;
    fs.release();
    return true;
}

//remove the unnecessary pointcloud of background
void RemoveBackground(PointCloudT::Ptr& cloudin, PointCloudT::Ptr& cloudout)
{
    cloudout->clear();
    for (PointCloudT::iterator iter = cloudin->begin(); iter != cloudin->end(); iter++)
    {
        if (iter->z > 0 && iter->y < 0 && iter->x < 1)
        {
            cloudout->push_back(*iter);
        }
    }
}

//ectract pointcloud of robot from original pointcloud using region growing
void ExtractRobot(PointCloudT::Ptr& origin_cloud, PointCloudT::Ptr& robot_cloud, PointCloudT::Ptr& remain_cloud)
{
    // Creating the KdTree object for the search method of the extraction
    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    float tolerance = 0.11896;
    pcl::PointXYZ point(0.0,0.0,0.0);
    origin_cloud->push_back(point);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(origin_cloud);

    // Check if the tree is sorted -- if it is we don't need to check the first element
    int nn_start_idx = tree->getSortedResults() ? 1 : 0;
    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed(origin_cloud->points.size(), false);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    std::vector<int>& seed_queue = inliers->indices;
    seed_queue.push_back(static_cast<int>(origin_cloud->points.size())-1);
    int sq_idx = 0;

    while (sq_idx < static_cast<int> (seed_queue.size()))
    {
        if (origin_cloud->points[seed_queue[sq_idx]].y < -0.6)
        {
            sq_idx++;
            continue;
        }

        // Search for sq_idx
        if (!tree->radiusSearch(origin_cloud->points[seed_queue[sq_idx]], tolerance, nn_indices, nn_distances))
        {
            sq_idx++;
            continue;
        }
        for (size_t j = nn_start_idx; j < nn_indices.size();++j)
        {
            if (nn_indices[j] == -1 || processed[nn_indices[j]])
                continue;

            // Perform a simple Euclidean clustering
            seed_queue.push_back(nn_indices[j]);
            processed[nn_indices[j]] = true;
        }
        sq_idx++;
    }

    //Extract new point cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(origin_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*robot_cloud);
    extract.setNegative(true);
    extract.filter(*remain_cloud);

}

//segment platform and box by setting the threshold of the Z direction (not a good idea)
//maybe, we can recongize the maximum plane in the pointcloud to extract platform
void SegmentPlaneBox(PointCloudT::Ptr& origin_cloud, PointCloudT::Ptr& plane_cloud, PointCloudT::Ptr& box_cloud)
{
    plane_cloud->clear();
    box_cloud->clear();
    for (PointCloudT::iterator iter = origin_cloud->begin(); iter != origin_cloud->end(); iter++)
    {
        if (iter->z < 0.06) {
            plane_cloud->push_back(*iter);
        }
        else{
            box_cloud->push_back(*iter);
        }
    }

}

//remove noise in the pointcloud
void RemoveNoisy(PointCloudT::Ptr& cloudin, PointCloudT::Ptr& cloudout, int meank)
{
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloudin);
    sor.setMeanK(meank);
    sor.setStddevMulThresh(0.5);
    sor.filter(*cloudout);
}
//Make the point cloud sparse to improve processing speed
void VoxelGridFilter(PointCloudT::Ptr& cloudin, PointCloudT::Ptr& cloudout, float param)
{
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(param, param, param);
    voxel.setInputCloud(cloudin);
    voxel.filter(*cloudout);
}


int main()
{
    std::string cloud_file_name("./data/cloud_origin.pcd");
    std::string matrix_file_name("./data/EyeHand.yaml");

    //load pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(cloud_file_name, *cloud_in))
    {
        std::cerr << "ERROR: Cannot open file " << cloud_file_name << "! Aborting..." << std::endl;
        return (-1);
    }
    std::cout << cloud_in->points.size() << std::endl;

    //read eye_to_hand matrix and transform pointcloud to the base coordinate system of the robot
    Eigen::Matrix4d transformation_matrix,transformation_matrix_inverse;
    ReadMatrix(matrix_file_name, transformation_matrix);
    std::cout << transformation_matrix << std::endl;
    transformation_matrix_inverse = transformation_matrix.inverse();
    pcl::transformPointCloud(*cloud_in, *cloud_in, transformation_matrix_inverse);

    //extract robot, platform and box form orignal pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remove(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr remain_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr robot_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    RemoveBackground(cloud_in, cloud_remove);
    ExtractRobot(cloud_remove,robot_cloud,remain_cloud);
    //RemoveNoisy(remain_cloud, remain_cloud, 5);
    SegmentPlaneBox(remain_cloud, plane_cloud, box_cloud);
    RemoveNoisy(plane_cloud, plane_cloud, 30);
    RemoveNoisy(box_cloud, box_cloud, 30);

    //save pointcloud
    //pcl::io::savePCDFile("./data/robot.pcd", *robot_cloud);
    //pcl::io::savePCDFile("./data/plane.pcd", *plane_cloud);
    //pcl::io::savePCDFile("./data/box.pcd", *box_cloud);

    //compute the plane equation of the platform
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;// Create the segmentation object
    seg.setOptimizeCoefficients(true);// Optional
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);
    seg.setInputCloud(plane_cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return (-1);
    }

    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudbox(new pcl::PointCloud<pcl::PointXYZ>());
    //pcl::ExtractIndices<pcl::PointXYZ> extract;
    //extract.setInputCloud(plane_cloud);
    //extract.setIndices(inliers);
    //extract.setNegative(true);
    //extract.filter(*cloudbox);

    //caculate the transform matrix that can make the normal vector of a plane coincide with the Z axis
    Eigen::Vector3d plane,transaxle;
    plane(0) = coefficients->values[0];
    plane(1) = coefficients->values[1];
    plane(2) = coefficients->values[2];
    Eigen::Vector3d axle(0, 0, 1);
    transaxle = plane.cross(axle);
    double transangle;
    transangle = acos(plane(2)/plane.norm());
    Eigen::AngleAxisd V1(transangle, transaxle);

    //rotate the pointcloud of box
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = V1.matrix();
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*box_cloud, *transformedCloud, transform);

    //compute the minimum bounding box
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(transformedCloud);
    feature_extractor.compute();
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);

    //obtain the top four points
    std::vector<pcl::PointXYZ> vertexpoint(4);
    vertexpoint[0].x = min_point_AABB.x;
    vertexpoint[1].x = max_point_AABB.x;
    vertexpoint[2].x = max_point_AABB.x;
    vertexpoint[3].x = min_point_AABB.x;
    vertexpoint[0].y = max_point_AABB.y;
    vertexpoint[1].y = max_point_AABB.y;
    vertexpoint[2].y = min_point_AABB.y;
    vertexpoint[3].y = min_point_AABB.y;
    vertexpoint[0].z = max_point_AABB.z;
    vertexpoint[1].z = max_point_AABB.z;
    vertexpoint[2].z = max_point_AABB.z;
    vertexpoint[3].z = max_point_AABB.z;

    //save as a point cloud for display
    PointCloudT::Ptr point_cloud(new PointCloudT);
    point_cloud->height = 1;
    point_cloud->width = 4;
    point_cloud->is_dense = false;
    point_cloud->points.resize(point_cloud->width * point_cloud->height);
    for (int i = 0; i < 4; ++i)
    {
        point_cloud->push_back(vertexpoint[i]);
    }

    //search the nearest K points and caculate mean value by kd_tree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(transformedCloud);
    int K = 5;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    pcl::PointXYZ point(0,0,0);
    for (int i = 0; i < 4; i++) {
        if (kdtree.nearestKSearch(vertexpoint[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            for (int j = 0; j < K; j++) {
                point.x += transformedCloud->points[pointIdxNKNSearch[j]].x;
                point.y += transformedCloud->points[pointIdxNKNSearch[j]].y;
                point.z += transformedCloud->points[pointIdxNKNSearch[j]].z;
            }
            point.x /= K;
            point.y /= K;
            point.z /= K;

            vertexpoint.push_back(point);
            point_cloud->push_back(point);
            pointIdxNKNSearch.clear();
            pointNKNSquaredDistance.clear();
            point.x = 0.0;
            point.y = 0.0;
            point.z = 0.0;
        }
    }

    //caculate the the coordinate of the coordinate axis
    Eigen::Vector3d box_frame_y, box_frame_x;
    Eigen::Vector3d point_vec,point_vec1, point_vec2;
    Eigen::Vector3d base_frame_y(0,1,0);
    Eigen::Vector3d base_frame_x(1,0,0);
    point_vec1(0) = double(vertexpoint[5].x-vertexpoint[4].x);
    point_vec1(1) = double(vertexpoint[5].y-vertexpoint[4].y);
    point_vec1(2) = 0.0;
    point_vec2(0) = double(vertexpoint[7].x-vertexpoint[4].x);
    point_vec2(1) = double(vertexpoint[7].y-vertexpoint[4].y);
    point_vec2(2) = 0.0;
    point_vec = point_vec1 + point_vec2;
    box_frame_x = point_vec/point_vec.norm();
    box_frame_y << 0,0,-1;

    //caculate two rotation matrices
    transaxle = box_frame_x.cross(base_frame_x);
    transangle = acos(box_frame_x(0)/box_frame_x.norm());
    Eigen::AngleAxisd transV1(transangle, transaxle);
    box_frame_y = transV1.matrix()*box_frame_y;
    transaxle = box_frame_y.cross(base_frame_y);
    transangle = acos(box_frame_y(1)/box_frame_y.norm());
    Eigen::AngleAxisd transV2(transangle, transaxle);

    //obtain the transform matrix
    Eigen::Matrix4d transform_box = Eigen::Matrix4d::Identity();
    transform_box.block(0, 0, 3, 3) = transV2.matrix()*transV1.matrix();
    Eigen::Vector3d translation_box(-vertexpoint[4].x,-vertexpoint[4].y,-vertexpoint[4].z);
    translation_box = transV2.matrix()*transV1.matrix()*translation_box;
    transform_box.block(0, 3, 3, 1) = translation_box;
    Eigen::Matrix4d transform_box_inverse = transform_box.inverse();

    Eigen::Matrix4d transform_all = transform_box*transform;

    //test quaterniond provided by robot
    /*Eigen::Quaterniond tempQ(0.26952,0.286332,0.619713,0.679211);
    Eigen::Matrix3d tempMax = tempQ.toRotationMatrix();
    Eigen::Matrix3d tempMax1;
    tempMax1 << -1,0,0,0,-1,0,0,0,1;
    tempMax = tempMax*tempMax1.inverse();
    transform_all.block(0, 0, 3, 3) = tempMax.inverse();
    transform_all.block(0, 3, 3, 1) = tempMax.inverse()*translation_box;*/

    //transform the pointcloud and obtain the pose matrix
    pcl::transformPointCloud(*box_cloud, *transformedCloud, transform_all);
    Eigen::Matrix4d transform_inverse = transform.inverse();
    pcl::transformPointCloud(*point_cloud, *point_cloud, transform_inverse);
    Eigen::Matrix4d box_pose = transform_all.inverse();
    SaveMatrix("./data/BoxPose.yaml", box_pose);

    //transform the pose matirx to quaterniond
    Eigen::Matrix3d box_rotation = box_pose.block(0, 0, 3, 3);
    Eigen::Quaterniond Pose(box_rotation);

    //output the pose matrix
    std::cout << "box_pose: " << std::endl;
    std::cout << box_pose << std::endl;

    //output the quaternion
    std::cout << "Quaterniond(x,y,z,w)" << std::endl;
    std::cout << Pose.x() << std::endl;
    std::cout << Pose.y() << std::endl;
    std::cout << Pose.z() << std::endl;
    std::cout << Pose.w() << std::endl;

    //save the top four points
    ofstream fsin;
    fsin.open("./data/points.txt",ios::trunc); //ios::trunc表示在打开文件前将文件清空,由于是写入,文件不存在则创建
    for(int i = 0; i < 4; i++){
        std::cout << "point" << i << " " << point_cloud->points[i+4].x << " " << point_cloud->points[i+4].y << " " << point_cloud->points[i+4].z << std::endl;
        fsin << point_cloud->points[i+4].x << " " << point_cloud->points[i+4].y << " " << point_cloud->points[i+4].z << "\n";
    }
    fsin.close();

    //display
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
    viewer->addPointCloud(transformedCloud, "trans_box");
    viewer->addPointCloud(box_cloud,"box");
    viewer->addCoordinateSystem(1.0);
    //viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
    //viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_h(point_cloud, 255, 0, 0);
    viewer->addPointCloud<PointT>(point_cloud, cloud_color_h, "point");
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}