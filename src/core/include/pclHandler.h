#ifndef __PCL_HANDLER_H__
#define __PCL_HANDLER_H__

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

#include <pcl/common/transforms.h>

#include <pcl/io/pcd_io.h>

///////////////////////
// SAC IA parameters //
///////////////////////
// Search Radius for Normals and FPFH features
const float SEARCH_RADIUS = 5.0f;

// Minimum distances between samples
const float MIN_SAMPLE_DISTANCE = 0.0f;
// Number of samples to use during each iteration
const int NR_SAMPLES = 3;
// Number of neighbors to use when selecting a random feature correspondence
const int CORRESPONDENCE_RANDOMNESS = 10;
// Maximum number of iterations the internal optimization should run for
const int MAX_ITERATIONS_SAC = 500;
// Inlier distance threshold for the internal RANSAC outlier rejection loop
const float OUTLIER_REJECTION_THRESHOLD_SAC = 5.0f; // 5mm
// Mximum distance threshold between two correspondent points in source <-> target.
const float MAX_CORRESPONDENCE_DISTANCE_SAC = 500.0f; // 500mmm  
// Maximum allowable translation squared difference between two consecutive transformations in order for an optimization to be considered as having converged.
const float TRANSFORMATION_EPSILON_SAC = 1e-3 * 1e-3; 
// Maximum allowed distance error before the algorithm will be considered to have converged
const float EUCLIDEAN_FITNESS_EPSILON_SAC = 100.0f;

////////////////////
// ICP parameters //
////////////////////
// Max Iterations before the algorithm will be considered to have converged
const int MAX_ITERATIONS_ICP = 25; 
// Mximum distance threshold between two correspondent points in source <-> target.
const float MAX_CORRESPONDENCE_DISTANCE_ICP = 150.0f; // 150mmm  
// Inlier distance threshold for the internal RANSAC outlier rejection loop
const float OUTLIER_REJECTION_THRESHOLD_ICP = 10.0f; // 10mm
// Maximum allowable translation squared difference between two consecutive transformations in order for an optimization to be considered as having converged.
const float TRANSFORMATION_EPSILON_ICP = 1e-10 * 1e-10; 
// Maximum allowable rotation difference between two consecutive transformations in order for an optimization to be considered as having converged.
const float ROTATION_EPSILON_ICP = 0.001; // 0.029deg
// Maximum allowed distance error before the algorithm will be considered to have converged
const float EUCLIDEAN_FITNESS_EPSILON_ICP = 1e-4;

typedef struct
{
    Eigen::RowVector2d translation_vector;
    double rotation_angle; // Rotation angle in degrees
} TransformationComponents;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudNormal;

typedef pcl::FPFHSignature33 LocalDescriptorT;
typedef pcl::PointCloud<LocalDescriptorT> LocalDescriptorCloud;

class PclHandler
{
public:
    PclHandler();
    TransformationComponents computeTransformation(Eigen::MatrixX2d first_scan, Eigen::MatrixX2d second_scan);

private:
    /* 
    * Use SampleConsensusInitialAlignment to find a rough alignment from the source cloud to the target cloud
    * Inputs:
    *   source_points
    *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
    *   target_points
    *     The "target" points, i.e., the points to which the source point cloud will be aligned
    * Return: A transformation matrix that will roughly align the points in source to the points in target
    */
    Eigen::Matrix4f computeInitialAlignment (const PointCloud::Ptr & source_points, const PointCloud::Ptr & target_points);

    /* 
    * Use IterativeClosestPoint to find a precise alignment from the source cloud to the target cloud
    * Inputs:
    *   source_points
    *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
    *   target_points
    *     The "target" points, i.e., the points to which the source point cloud will be aligned
    * Return: A transformation matrix that will precisely align the points in source to the points in target
    */
    Eigen::Matrix4f refineAlignment (const PointCloud::Ptr & source_points, const PointCloud::Ptr & target_points);

    PointCloud matrixToPointCloud(Eigen::MatrixX2d matrix);
    Eigen::RowVector3f polarToCartesianXYZ(Eigen::RowVector2d polar_point);
    TransformationComponents extractTransformationComponents(Eigen::Matrix4f transformation_matrix);
};

#endif //__PCL_HANDLER_H__