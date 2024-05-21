#include "pclHandler.h"

PclHandler::PclHandler()
{
}

TransformationComponents PclHandler::computeTransformation(Eigen::MatrixX2d first_scan, Eigen::MatrixX2d second_scan)
{
    PointCloudPtr source_cloud_ptr (new PointCloud);
    PointCloudPtr target_cloud_ptr (new PointCloud);

    PointCloud& source_cloud = *source_cloud_ptr;
    PointCloud& target_cloud = *target_cloud_ptr;

    source_cloud = matrixToPointCloud(first_scan);
    target_cloud = matrixToPointCloud(second_scan);

    // Remove NaN points from point clouds
    // (this is necessary to avoid a segfault when running ICP)
    std::vector<int> nan_idx;
    pcl::removeNaNFromPointCloud(source_cloud, source_cloud, nan_idx);
    pcl::removeNaNFromPointCloud(target_cloud, target_cloud, nan_idx);

    /* Compute the transformation matrix for alignment
    Eigen::Matrix4f tform = Eigen::Matrix4f::Identity();
    tform = computeInitialAlignment (src_keypoints_ptr, src_features_ptr, tar_keypoints_ptr,
            tar_features_ptr, min_sample_dist, max_correspondence_dist, nr_iters);
    */

    // Uncomment this code to run ICP 
    Eigen::Matrix4f tform = Eigen::Matrix4f::Identity();
    tform = refineAlignment (source_cloud_ptr, target_cloud_ptr, max_correspondence_distance,
            outlier_rejection_threshold, transformation_epsilon, max_iterations);
    
    
    std::cout << "Calculated transformation\n";

    return extractTransformationComponents(tform);
}

Eigen::Matrix4f PclHandler::refineAlignment (const ICPPointCloudPtr & source_points, const ICPPointCloudPtr & target_points,
                                             float max_correspondence_distance, float outlier_rejection_threshold, 
                                             float transformation_epsilon, float max_iterations) 
{

    pcl::IterativeClosestPoint<ICPPointT, ICPPointT> icp;
    icp.setMaxCorrespondenceDistance (max_correspondence_distance);
    icp.setRANSACOutlierRejectionThreshold (outlier_rejection_threshold);
    icp.setTransformationEpsilon (transformation_epsilon);
    icp.setMaximumIterations (max_iterations);

    ICPPointCloudPtr source_points_icp (new ICPPointCloud);
    source_points_icp = source_points;

    icp.setInputSource (source_points_icp);
    icp.setInputTarget (target_points);

    ICPPointCloud registration_output;
    icp.align (registration_output);

    return icp.getFinalTransformation ();
}

PointCloud PclHandler::matrixToPointCloud(Eigen::MatrixX2d matrix)
{
    PointCloud point_cloud;
    point_cloud.resize(matrix.rows());

    Eigen::RowVector3i point_vector;

    for (int i = 0; i < matrix.rows(); i++)
    {
        point_vector = polarToCartesianXYZ(matrix.row(i));
        point_cloud[i].x = (float)point_vector[0];
        point_cloud[i].y = (float)point_vector[1];
        point_cloud[i].z = (float)point_vector[2];
    }

    return point_cloud;
}

Eigen::RowVector3i PclHandler::polarToCartesianXYZ(Eigen::RowVector2d polar_point)
{
    Eigen::RowVector3i cart_point;

    double theta = polar_point[0];
    double r = polar_point[1];

    cart_point[0] = round(r * cos(theta));
    cart_point[1] = round(r * sin(theta));
    cart_point[2] = 0;

    return cart_point;
}

TransformationComponents PclHandler::extractTransformationComponents(Eigen::Matrix4f transformation_matrix)
{
    TransformationComponents components;

    // Extract translation vector with only x and y as double
    components.translation_vector << static_cast<double>(transformation_matrix(0, 3)),
        static_cast<double>(transformation_matrix(1, 3));

    // Calculate rotation angle from the rotation matrix part of T
    // Assuming the rotation is around the Z-axis
    double theta_rad = std::atan2(transformation_matrix(1, 0), transformation_matrix(0, 0)); // atan2(sin, cos)
    components.rotation_angle = theta_rad;                                                 // Store in radians

    return components;
}