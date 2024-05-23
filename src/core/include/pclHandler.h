#ifndef __PCL_HANDLER_H__
#define __PCL_HANDLER_H__

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter_indices.h>

// ICP parameters
const float max_correspondence_distance = 0.005f;  //  10 mm
const float outlier_rejection_threshold = 0.01f;   // 20 mm
const float transformation_epsilon = 0;
const int max_iterations = 300;

typedef struct
{
    Eigen::RowVector2d translation_vector;
    double rotation_angle; // Rotation angle in degrees
} TransformationComponents;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;

class PclHandler
{
public:
    PclHandler();
    TransformationComponents computeTransformation(Eigen::MatrixX2d first_scan, Eigen::MatrixX2d second_scan);

private:
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
    Eigen::Matrix4f computeAlignment (const PointCloudPtr & source_points, const PointCloudPtr & target_points,
                                     float max_correspondence_distance, float outlier_rejection_threshold, 
                                     float transformation_epsilon, float max_iterations);

    PointCloud matrixToPointCloud(Eigen::MatrixX2d matrix);
    Eigen::RowVector3f polarToCartesianXYZ(Eigen::RowVector2d polar_point);
    TransformationComponents extractTransformationComponents(Eigen::Matrix4f transformation_matrix);
};

#endif //__PCL_HANDLER_H__