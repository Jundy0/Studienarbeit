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

    // Run ICP 
    Eigen::Matrix4f tform = Eigen::Matrix4f::Identity();
    tform = computeAlignment (source_cloud_ptr, target_cloud_ptr, max_correspondence_distance,
            outlier_rejection_threshold, transformation_epsilon, max_iterations);
    
    std::cout << "Calculated transformation\n";

    return extractTransformationComponents(tform);
}

Eigen::Matrix4f PclHandler::computeAlignment (const PointCloudPtr & source_points, const PointCloudPtr & target_points,
                                             float max_correspondence_distance, float outlier_rejection_threshold, 
                                             float transformation_epsilon, float max_iterations) 
{

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaxCorrespondenceDistance (max_correspondence_distance);
    icp.setRANSACOutlierRejectionThreshold (outlier_rejection_threshold);
    icp.setTransformationEpsilon (transformation_epsilon);
    icp.setMaximumIterations (max_iterations);

    PointCloudPtr source_points_icp (new PointCloud);
    source_points_icp = source_points;

    icp.setInputSource (source_points_icp);
    icp.setInputTarget (target_points);

    PointCloud registration_output;
    icp.align (registration_output);

    std::cout << "Converged: " << icp.hasConverged() << ", Fitness: " << icp.getFitnessScore() << std::endl;

    return icp.getFinalTransformation ();
}

PointCloud PclHandler::matrixToPointCloud(Eigen::MatrixX2d matrix)
{
    PointCloud point_cloud;
    point_cloud.resize(matrix.rows());

    Eigen::RowVector3f point_vector;

    for (int i = 0; i < matrix.rows(); i++)
    {
        point_vector = polarToCartesianXYZ(matrix.row(i));
        point_cloud[i].x = point_vector[0];
        point_cloud[i].y = point_vector[1];
        point_cloud[i].z = point_vector[2];
    }

    return point_cloud;
}

Eigen::RowVector3f PclHandler::polarToCartesianXYZ(Eigen::RowVector2d polar_point)
{
    Eigen::RowVector3f cart_point;

    double theta = polar_point[0];
    double r = polar_point[1];

    cart_point[0] = r * cos(theta) / 1000;
    cart_point[1] = r * sin(theta) / 1000;
    cart_point[2] = 0;

    return cart_point;
}

TransformationComponents PclHandler::extractTransformationComponents(Eigen::Matrix4f transformation_matrix)
{
    TransformationComponents components;

    // Extract translation vector with only x and y as double
    components.translation_vector << static_cast<double>(transformation_matrix(0, 3) * 1000),
        static_cast<double>(transformation_matrix(1, 3) * 1000);

    // Calculate rotation angle from the rotation matrix part of T
    // Assuming the rotation is around the Z-axis
    double theta_rad = std::atan2(transformation_matrix(1, 0), transformation_matrix(0, 0)); // atan2(sin, cos)
    components.rotation_angle = theta_rad;                                                 // Store in radians

    return components;
}