#include "pclHandler.h"

PclHandler::PclHandler()
{
}

TransformationComponents PclHandler::computeTransformation(Eigen::MatrixX2d first_scan, Eigen::MatrixX2d second_scan, double current_rotation)
{
    // Create Point Clouds to write data to
    PointCloud::Ptr source_cloud_ptr(new PointCloud);
    PointCloud::Ptr target_cloud_ptr(new PointCloud);

    PointCloud &source_cloud = *source_cloud_ptr;
    PointCloud &target_cloud = *target_cloud_ptr;

    // Write data from matrix to point clouds
    source_cloud = matrixToPointCloud(first_scan, current_rotation);
    target_cloud = matrixToPointCloud(second_scan, current_rotation);

    //// DEBUG ONLY ////
    pcl::io::savePCDFileASCII("source.pcd", source_cloud);
    pcl::io::savePCDFileASCII("target.pcd", target_cloud);

    // Run ICP
    Eigen::Matrix4f tf_matrix = Eigen::Matrix4f::Identity();
    tf_matrix = computeAlignment(source_cloud_ptr, target_cloud_ptr);

    //// DEBUG ONLY ////    
    pcl::io::savePCDFileASCII("result_icp.pcd", source_cloud);

    // Return extracted TransformationComponents
    return extractTransformationComponents(tf_matrix);
}

Eigen::Matrix4f PclHandler::computeAlignment(const PointCloud::Ptr &source_points, const PointCloud::Ptr &target_points)
{
    // Create ICP instance
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    
    // Set registration output to the pointer of the source cloud
    // This overwrites the source cloud with the result of the icp so it can be used for another iteration
    PointCloud::Ptr registration_output = source_points;

    // Create initial transformation matrix that does nothing
    Eigen::Matrix4f tf_matrix;
    tf_matrix << 1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;

    // Loop ICP until fitness score is under a threshhold or other criteria is met
    do
    {
        // Set ICP parameter
        icp.setMaximumIterations(MAX_ITERATIONS_ICP);
        icp.setMaxCorrespondenceDistance(MAX_CORRESPONDENCE_DISTANCE_ICP);
        icp.setRANSACOutlierRejectionThreshold(OUTLIER_REJECTION_THRESHOLD_ICP);
        icp.setTransformationEpsilon(TRANSFORMATION_EPSILON_ICP);
        icp.setTransformationRotationEpsilon(ROTATION_EPSILON_ICP);
        icp.setEuclideanFitnessEpsilon(EUCLIDEAN_FITNESS_EPSILON_ICP);

        // Set ICP source and target point cloud
        icp.setInputSource(registration_output);
        icp.setInputTarget(target_points);

        // Run ICP
        icp.align(*registration_output);

        // Get transformation matrix and store it to tf_matrix
        // If icp is not run for the first time the result gets added onto the results from previous iterations
        tf_matrix = icp.getFinalTransformation() * tf_matrix;

    } while (icp.getFitnessScore() > 500);
    
    // Output of ICP results
    std::cout << "Converged: " << icp.hasConverged() << std::endl;
    std::cout << "Fitness: " << icp.getFitnessScore() << std::endl;
    std::cout << "Reason: " << icp.getConvergeCriteria()->getConvergenceState() << std::endl;

    return (tf_matrix);
}

PointCloud PclHandler::matrixToPointCloud(Eigen::MatrixX2d matrix, double current_rotation)
{
    // Create PointCloud with the same size as the input matrix
    PointCloud point_cloud;
    point_cloud.resize(matrix.rows());

    Eigen::RowVector3f point_vector;

    for (int i = 0; i < matrix.rows(); i++)
    {
        // Rotate point cloud and calculate X,Y and Z coordinates
        point_vector = polarToCartesianXYZ(matrix.row(i), current_rotation);
        point_cloud[i].x = point_vector[0];
        point_cloud[i].y = point_vector[1];
        point_cloud[i].z = point_vector[2];
    }

    return point_cloud;
}

Eigen::RowVector3f PclHandler::polarToCartesianXYZ(Eigen::RowVector2d polar_point, double current_rotation)
{
    Eigen::RowVector3f cart_point;

    double theta = polar_point[0];
    double r = polar_point[1];

    // Adds rotation to theta to align point cloud with global map axis
    cart_point[0] = (r)*cos(theta + current_rotation);
    cart_point[1] = (r)*sin(theta + current_rotation);
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
    double theta_rad = std::atan2(transformation_matrix(1, 0), transformation_matrix(0, 0)); // atan2(sin, cos)
    components.rotation_angle = -theta_rad;                                                  // Store in radians

    return components;
}
