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

    // Run SAC-IA
    Eigen::Matrix4f tform = Eigen::Matrix4f::Identity();
    tform = computeInitialAlignment (source_cloud_ptr, target_cloud_ptr);

    pcl::io::savePCDFileASCII ("result_sac_ia.pcd", source_cloud);

    // Run ICP
    tform = refineAlignment (source_cloud_ptr, target_cloud_ptr) * tform;

    pcl::io::savePCDFileASCII ("result_icp.pcd", source_cloud);

    std::cout << "Calculated transformation\n";

    return extractTransformationComponents(tform);
}

Eigen::Matrix4f PclHandler::computeInitialAlignment (const PointCloud::Ptr & source_points, const PointCloud::Ptr & target_points)
{
    // Estimate cloud normals
    std::cout << "Computing source cloud normals" << std::endl;
    pcl::NormalEstimation<PointT, PointNormalT> ne;
    PointCloudNormal::Ptr source_normals_ptr (new PointCloudNormal);
    PointCloudNormal& source_normals = *source_normals_ptr;

    pcl::search::KdTree<PointT>::Ptr tree_xyz (new pcl::search::KdTree<PointT>());

    ne.setInputCloud(source_points);
    ne.setSearchMethod(tree_xyz);
    ne.setRadiusSearch(SEARCH_RADIUS);
    ne.compute(*source_normals_ptr);
    for(size_t i = 0;  i < source_normals.points.size(); ++i) {
        source_normals.points[i].x = source_points->points[i].x;
        source_normals.points[i].y = source_points->points[i].y;
        source_normals.points[i].z = source_points->points[i].z;
    }

    std::cout << "Computing target cloud normals" << std::endl;
    PointCloudNormal::Ptr target_normals_ptr (new PointCloudNormal);
    PointCloudNormal& target_normals = *target_normals_ptr;

    ne.setInputCloud(target_points);
    ne.compute(*target_normals_ptr);
    for(size_t i = 0;  i < target_normals.points.size(); ++i) {
        target_normals.points[i].x = target_points->points[i].x;
        target_normals.points[i].y = target_points->points[i].y;
        target_normals.points[i].z = target_points->points[i].z;
    }

    // Extract FPFH features
    // Source
    PointCloud::Ptr source_xyz (new PointCloud);                           
    pcl::copyPointCloud (*source_points, *source_xyz);
    pcl::FPFHEstimation<PointT, PointNormalT, LocalDescriptorT> fpfh;

    fpfh.setSearchSurface (source_points);
    fpfh.setInputCloud (source_xyz);
    fpfh.setInputNormals (source_normals_ptr);
    fpfh.setSearchMethod (tree_xyz);

    pcl::PointCloud<LocalDescriptorT>::Ptr source_features_ptr (new pcl::PointCloud<LocalDescriptorT>());
    pcl::PointCloud<LocalDescriptorT>& source_features = *source_features_ptr;

    fpfh.setRadiusSearch(SEARCH_RADIUS);
    fpfh.compute(source_features);
    std::cout << "Computed " << source_features.size() << " FPFH features for source cloud" << std::endl;

    // Target
    PointCloud::Ptr target_xyz (new PointCloud);                           
    pcl::copyPointCloud (*target_points, *target_xyz);

    fpfh.setSearchSurface (target_points);
    fpfh.setInputCloud (target_xyz);
    fpfh.setInputNormals (target_normals_ptr);
    pcl::PointCloud<LocalDescriptorT>::Ptr target_features_ptr (new pcl::PointCloud<LocalDescriptorT>());
    pcl::PointCloud<LocalDescriptorT>& target_features = *target_features_ptr;
    fpfh.compute(target_features);
    std::cout << "Computed " << target_features.size() << " FPFH features for target cloud" << std::endl;

    // SAC-IA
    pcl::SampleConsensusInitialAlignment<PointT, PointT, LocalDescriptorT> sac_ia;

    sac_ia.setMinSampleDistance (MIN_SAMPLE_DISTANCE);
    sac_ia.setNumberOfSamples (NR_SAMPLES);
    sac_ia.setCorrespondenceRandomness (CORRESPONDENCE_RANDOMNESS);

    sac_ia.setMaximumIterations (MAX_ITERATIONS_SAC);
    sac_ia.setRANSACOutlierRejectionThreshold (OUTLIER_REJECTION_THRESHOLD_SAC);
    sac_ia.setMaxCorrespondenceDistance (MAX_CORRESPONDENCE_DISTANCE_SAC);
    sac_ia.setTransformationEpsilon(TRANSFORMATION_EPSILON_SAC);
    sac_ia.setEuclideanFitnessEpsilon (EUCLIDEAN_FITNESS_EPSILON_SAC);

    sac_ia.setInputSource (source_points);
    sac_ia.setSourceFeatures (source_features_ptr);

    sac_ia.setInputTarget (target_points);
    sac_ia.setTargetFeatures (target_features_ptr);

    PointCloud::Ptr registration_output = source_points;
    sac_ia.align (*registration_output);

    std::cout << "Converged: " << sac_ia.hasConverged() << ", Fitness: " << sac_ia.getFitnessScore() << std::endl;

    return (sac_ia.getFinalTransformation ());
}


Eigen::Matrix4f PclHandler::refineAlignment (const PointCloud::Ptr & source_points, const PointCloud::Ptr & target_points) 
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

    if (icp.getFitnessScore() > 100)
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
    components.translation_vector << static_cast<double>(transformation_matrix(0, 3)),
        static_cast<double>(transformation_matrix(1, 3));

    // Calculate rotation angle from the rotation matrix part of T
    // Assuming the rotation is around the Z-axis
    double theta_rad = std::atan2(transformation_matrix(1, 0), transformation_matrix(0, 0));    // atan2(sin, cos)
    components.rotation_angle = theta_rad;                                                      // Store in radians

    return components;
}