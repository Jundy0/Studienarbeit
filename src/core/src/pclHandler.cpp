#include "pclHandler.h"

PclHandler::PclHandler()
{
}

TransformationComponents PclHandler::computeTransformation(Eigen::MatrixX2d first_scan, Eigen::MatrixX2d second_scan)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>& source_cloud = *source_cloud_ptr;
    pcl::PointCloud<pcl::PointXYZ>& target_cloud = *target_cloud_ptr;

    source_cloud = matrixToPointCloudPtr(first_scan);
    target_cloud = matrixToPointCloudPtr(second_scan);

    // Estimate cloud normals
    std::cout << "Computing source cloud normals\n";
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    pcl::PointCloud<pcl::PointNormal>::Ptr src_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>& src_normals = *src_normals_ptr;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_xyz (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setInputCloud(source_cloud_ptr);
    ne.setSearchMethod(tree_xyz);
    ne.setRadiusSearch(0.05);
    ne.compute(*src_normals_ptr);
    for(size_t i = 0;  i < src_normals.points.size(); ++i) {
        src_normals.points[i].x = source_cloud.points[i].x;
        src_normals.points[i].y = source_cloud.points[i].y;
        src_normals.points[i].z = source_cloud.points[i].z;
    }

    std::cout << "Computing target cloud normals\n";
    pcl::PointCloud<pcl::PointNormal>::Ptr tar_normals_ptr (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>& tar_normals = *tar_normals_ptr;
    ne.setInputCloud(target_cloud_ptr);
    ne.compute(*tar_normals_ptr);
    for(size_t i = 0;  i < tar_normals.points.size(); ++i) {
        tar_normals.points[i].x = target_cloud.points[i].x;
        tar_normals.points[i].y = target_cloud.points[i].y;
        tar_normals.points[i].z = target_cloud.points[i].z;
    }

    // Estimate the SIFT keypoints
    pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale>::Ptr src_keypoints_ptr (new pcl::PointCloud<pcl::PointWithScale>);
    pcl::PointCloud<pcl::PointWithScale>& src_keypoints = *src_keypoints_ptr;
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree_normal(new pcl::search::KdTree<pcl::PointNormal> ());
    sift.setSearchMethod(tree_normal);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(src_normals_ptr);
    sift.compute(src_keypoints);

    std::cout << "Found " << src_keypoints.points.size () << " SIFT keypoints in source cloud\n";
    
    pcl::PointCloud<pcl::PointWithScale>::Ptr tar_keypoints_ptr (new pcl::PointCloud<pcl::PointWithScale>);
    pcl::PointCloud<pcl::PointWithScale>& tar_keypoints = *tar_keypoints_ptr;
    sift.setInputCloud(tar_normals_ptr);
    sift.compute(tar_keypoints);

    std::cout << "Found " << tar_keypoints.points.size () << " SIFT keypoints in target cloud\n";

    // Extract FPFH features from SIFT keypoints
    pcl::PointCloud<pcl::PointXYZ>::Ptr src_keypoints_xyz (new pcl::PointCloud<pcl::PointXYZ>);                           
    pcl::copyPointCloud (src_keypoints, *src_keypoints_xyz);
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
    fpfh.setSearchSurface (source_cloud_ptr);
    fpfh.setInputCloud (src_keypoints_xyz);
    fpfh.setInputNormals (src_normals_ptr);
    fpfh.setSearchMethod (tree_xyz);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr src_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::FPFHSignature33>& src_features = *src_features_ptr;
    fpfh.setRadiusSearch(0.05);
    fpfh.compute(src_features);
    std::cout << "Computed " << src_features.size() << " FPFH features for source cloud\n";

    pcl::PointCloud<pcl::PointXYZ>::Ptr tar_keypoints_xyz (new pcl::PointCloud<pcl::PointXYZ>);                           
    pcl::copyPointCloud (tar_keypoints, *tar_keypoints_xyz);
    fpfh.setSearchSurface (target_cloud_ptr);
    fpfh.setInputCloud (tar_keypoints_xyz);
    fpfh.setInputNormals (tar_normals_ptr);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr tar_features_ptr (new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::FPFHSignature33>& tar_features = *tar_features_ptr;
    fpfh.compute(tar_features);
    std::cout << "Computed " << tar_features.size() << " FPFH features for target cloud\n";
    
    // Compute the transformation matrix for alignment
    Eigen::Matrix4f tform = Eigen::Matrix4f::Identity();
    tform = computeInitialAlignment (src_keypoints_ptr, src_features_ptr, tar_keypoints_ptr,
            tar_features_ptr, min_sample_dist, max_correspondence_dist, nr_iters);

    /* Uncomment this code to run ICP 
    tform = refineAlignment (source_cloud_ptr, target_cloud_ptr, tform, max_correspondence_distance,
            outlier_rejection_threshold, transformation_epsilon, max_iterations);
    */
    
    std::cout << "Calculated transformation\n";

    return extractTransformationComponents(tform);
}

Eigen::Matrix4f PclHandler::computeInitialAlignment (const PointCloudPtr & source_points, const LocalDescriptorsPtr & source_descriptors,
                                         const PointCloudPtr & target_points, const LocalDescriptorsPtr & target_descriptors,
                                         float min_sample_distance, float max_correspondence_distance, int nr_iterations)
{
    pcl::SampleConsensusInitialAlignment<PointT, PointT, LocalDescriptorT> sac_ia;
    sac_ia.setMinSampleDistance (min_sample_distance);
    sac_ia.setMaxCorrespondenceDistance (max_correspondence_distance);
    sac_ia.setMaximumIterations (nr_iterations);

    sac_ia.setInputSource (source_points);
    sac_ia.setSourceFeatures (source_descriptors);

    sac_ia.setInputTarget (target_points);
    sac_ia.setTargetFeatures (target_descriptors);

    PointCloud registration_output;
    sac_ia.align (registration_output);

    return (sac_ia.getFinalTransformation ());
}

Eigen::Matrix4f PclHandler::refineAlignment (const ICPPointCloudPtr & source_points, const ICPPointCloudPtr & target_points,
                                     const Eigen::Matrix4f initial_alignment, float max_correspondence_distance,
                                     float outlier_rejection_threshold, float transformation_epsilon, float max_iterations) 
{

    pcl::IterativeClosestPoint<ICPPointT, ICPPointT> icp;
    icp.setMaxCorrespondenceDistance (max_correspondence_distance);
    icp.setRANSACOutlierRejectionThreshold (outlier_rejection_threshold);
    icp.setTransformationEpsilon (transformation_epsilon);
    icp.setMaximumIterations (max_iterations);

    ICPPointCloudPtr source_points_transformed (new ICPPointCloud);
    pcl::transformPointCloud (*source_points, *source_points_transformed, initial_alignment);

    icp.setInputSource (source_points_transformed);
    icp.setInputTarget (target_points);

    ICPPointCloud registration_output;
    icp.align (registration_output);

    return (icp.getFinalTransformation () * initial_alignment);
}

pcl::PointCloud<pcl::PointXYZ> PclHandler::matrixToPointCloudPtr(Eigen::MatrixX2d matrix)
{
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
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
    components.translation_vector << static_cast<double>(transformation_matrix(0, 2)),
        static_cast<double>(transformation_matrix(1, 2));

    // Calculate rotation angle from the rotation matrix part of T
    // Assuming the rotation is around the Z-axis
    double theta_rad = std::atan2(transformation_matrix(1, 0), transformation_matrix(0, 0)); // atan2(sin, cos)
    components.rotation_angle = theta_rad;                                                 // Store in radians

    return components;
}