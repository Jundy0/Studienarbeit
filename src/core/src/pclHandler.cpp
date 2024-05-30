#include "pclHandler.h"

PclHandler::PclHandler()
{
}

TransformationComponents PclHandler::computeTransformation(Eigen::MatrixX2d first_scan, Eigen::MatrixX2d second_scan)
{
    PointCloud::Ptr source_cloud_ptr (new PointCloud);
    PointCloud::Ptr target_cloud_ptr (new PointCloud);

    PointCloud& source_cloud = *source_cloud_ptr;
    PointCloud& target_cloud = *target_cloud_ptr;

    source_cloud = matrixToPointCloud(first_scan);
    target_cloud = matrixToPointCloud(second_scan);

    ///////////////////

    pcl::io::savePCDFileASCII ("source.pcd", source_cloud);
    pcl::io::savePCDFileASCII ("target.pcd", target_cloud);

    ///////////////////

    // Run ICP
    Eigen::Matrix4f tform = Eigen::Matrix4f::Identity();
    tform = computeAlignment (source_cloud_ptr, target_cloud_ptr);

    pcl::io::savePCDFileASCII ("result_icp.pcd", source_cloud);

    std::cout << "Calculated transformation\n";

    return extractTransformationComponents(tform);
}

Eigen::Matrix4f PclHandler::computeAlignment (const PointCloud::Ptr & source_points, const PointCloud::Ptr & target_points) 
{
    pcl::IterativeClosestPoint<PointT, PointT> icp;

    icp.setMaximumIterations (MAX_ITERATIONS_ICP);
    icp.setMaxCorrespondenceDistance (MAX_CORRESPONDENCE_DISTANCE_ICP);
    icp.setRANSACOutlierRejectionThreshold (OUTLIER_REJECTION_THRESHOLD_ICP);
    icp.setTransformationEpsilon (TRANSFORMATION_EPSILON_ICP);
    icp.setTransformationRotationEpsilon(ROTATION_EPSILON_ICP);
    icp.setEuclideanFitnessEpsilon(EUCLIDEAN_FITNESS_EPSILON_ICP);
    
    icp.setInputSource (source_points);
    icp.setInputTarget (target_points);

    PointCloud::Ptr registration_output = source_points;
    icp.align (*registration_output);

    std::cout << "Converged: " << icp.hasConverged() << ", Fitness: " << icp.getFitnessScore() << std::endl;

    if (icp.getFitnessScore() > 500)
    {
        std::cout << "Bad Fitness" << std::endl;
    }

    return (icp.getFinalTransformation ());
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

    cart_point[0] = (r) * cos(theta);
    cart_point[1] = (r) * sin(theta);
    cart_point[2] = 0;

    return cart_point;
}

TransformationComponents PclHandler::extractTransformationComponents(Eigen::Matrix4f transformation_matrix)
{
    TransformationComponents components;

    // Extract translation vector with only x and y as double
    components.translation_vector << static_cast<double>(-transformation_matrix(0, 3)),
                                     static_cast<double>(-transformation_matrix(1, 3));

    // Calculate rotation angle from the rotation matrix part of T
    // Assuming the rotation is around the Z-axis
    double theta_rad = std::atan2(transformation_matrix(1, 0), transformation_matrix(0, 0));    // atan2(sin, cos)
    components.rotation_angle = -theta_rad;                                                     // Store in radians

    return components;
}