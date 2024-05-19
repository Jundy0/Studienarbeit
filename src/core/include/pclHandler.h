#ifndef __PCL_HANDLER_H__
#define __PCL_HANDLER_H__

#include <pcl/point_types.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

// --------------------
// -----Parameters-----
// --------------------
// SIFT Keypoint parameters
const float min_scale = 0.01f; // the standard deviation of the smallest scale in the scale space
const int n_octaves = 3;  // the number of octaves (i.e. doublings of scale) to compute
const int n_scales_per_octave = 4; // the number of scales to compute within each octave
const float min_contrast = 0.001f; // the minimum contrast required for detection

// Sample Consensus Initial Alignment parameters (explanation below)
const float min_sample_dist = 0.025f;
const float max_correspondence_dist = 0.01f;
const int nr_iters = 500;

// ICP parameters (explanation below)
const float max_correspondence_distance = 0.05f;
const float outlier_rejection_threshold = 0.05f;
const float transformation_epsilon = 0;
const int max_iterations = 100;

typedef struct
{
    Eigen::RowVector2d translation_vector;
    double rotation_angle; // Rotation angle in degrees
} TransformationComponents;

class PclHandler
{
public:
    PclHandler();
    TransformationComponents computeTransformation(Eigen::MatrixX2d first_scan, Eigen::MatrixX2d second_scan);

private:
    /* Use SampleConsensusInitialAlignment to find a rough alignment from the source cloud to the target cloud by fin    ding
    * correspondences between two sets of local features
    * Inputs:
    *   source_points
    *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
    *   source_descriptors
    *     The local descriptors for each source point
    *   target_points
    *     The "target" points, i.e., the points to which the source point cloud will be aligned                     
    *   target_descriptors
    *     The local descriptors for each target point
    *   min_sample_distance
    *     The minimum distance between any two random samples
    *   max_correspondence_distance
    *     The maximum distance between a point and its nearest neighbor correspondent in order to be considered
    *     in the alignment process
    *   nr_interations
    *     The number of RANSAC iterations to perform
    * Return: A transformation matrix that will roughly align the points in source to the points in target
    */
    typedef pcl::PointWithScale PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
    typedef pcl::FPFHSignature33 LocalDescriptorT;
    typedef pcl::PointCloud<LocalDescriptorT>::Ptr LocalDescriptorsPtr;
    Eigen::Matrix4f computeInitialAlignment (const PointCloudPtr & source_points, const LocalDescriptorsPtr & source_descriptors,
                                             const PointCloudPtr & target_points, const LocalDescriptorsPtr & target_descriptors,
                                             float min_sample_distance, float max_correspondence_distance, int nr_iterations);

    /* Use IterativeClosestPoint to find a precise alignment from the source cloud to the target cloud,                   
    * starting with an intial guess
    * Inputs:
    *   source_points
    *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
    *   target_points
    *     The "target" points, i.e., the points to which the source point cloud will be aligned
    *   intial_alignment
    *     An initial estimate of the transformation matrix that aligns the source points to the target points
    *   max_correspondence_distance
    *     A threshold on the distance between any two corresponding points.  Any corresponding points that are further 
    *     apart than this threshold will be ignored when computing the source-to-target transformation
    *   outlier_rejection_threshold
    *     A threshold used to define outliers during RANSAC outlier rejection
    *   transformation_epsilon
    *     The smallest iterative transformation allowed before the algorithm is considered to have converged
    *   max_iterations
    *     The maximum number of ICP iterations to perform
    * Return: A transformation matrix that will precisely align the points in source to the points in target
    */
    typedef pcl::PointXYZ ICPPointT;
    typedef pcl::PointCloud<ICPPointT> ICPPointCloud;
    typedef pcl::PointCloud<ICPPointT>::Ptr ICPPointCloudPtr;
    Eigen::Matrix4f refineAlignment (const ICPPointCloudPtr & source_points, const ICPPointCloudPtr & target_points,
                                     const Eigen::Matrix4f initial_alignment, float max_correspondence_distance,
                                     float outlier_rejection_threshold, float transformation_epsilon, float max_iterations);

    pcl::PointCloud<pcl::PointXYZ> matrixToPointCloudPtr(Eigen::MatrixX2d matrix);
    Eigen::RowVector3i polarToCartesianXYZ(Eigen::RowVector2d polar_point);
    TransformationComponents extractTransformationComponents(Eigen::Matrix4f transformation_matrix);
};

#endif //__PCL_HANDLER_H__