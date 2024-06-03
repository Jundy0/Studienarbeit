#ifndef __PCL_HANDLER_H__
#define __PCL_HANDLER_H__

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <pcl/common/transforms.h>

#include <pcl/io/pcd_io.h>

////////////////////
// ICP parameters //
////////////////////
/// @brief Max Iterations before the algorithm will be considered to have converged
const int MAX_ITERATIONS_ICP = 25;
/// @brief Mximum distance threshold between two correspondent points in source <-> target.
const float MAX_CORRESPONDENCE_DISTANCE_ICP = 150.0f; // 150mmm
/// @brief Inlier distance threshold for the internal RANSAC outlier rejection loop
const float OUTLIER_REJECTION_THRESHOLD_ICP = 10.0f; // 10mm
/// @brief Maximum allowable translation squared difference between two consecutive transformations in order for an optimization to be considered as having converged.
const float TRANSFORMATION_EPSILON_ICP = 1e-12 * 1e-12;
/// @brief Maximum allowable rotation difference between two consecutive transformations in order for an optimization to be considered as having converged.
const float ROTATION_EPSILON_ICP = 1e-5; // 0.029deg
/// @brief Maximum allowed distance error before the algorithm will be considered to have converged
const float EUCLIDEAN_FITNESS_EPSILON_ICP = 1e-10;

/// @brief The transformation components extracted from the transformation matrix got using icp
typedef struct
{
    /// @brief A vector composed of global x and y values
    Eigen::RowVector2d translation_vector;
    /// @brief A rotation angle in deg
    double rotation_angle;
} TransformationComponents;

/// @brief Point definition used during icp computation
typedef pcl::PointXYZ PointT;
/// @brief PointCloud definition used during icp computation
typedef pcl::PointCloud<PointT> PointCloud;

/// @brief Interface between particle and the pcl library
class PclHandler
{
public:
    /// @brief Empty Constructor.
    PclHandler();
    /// @brief Computes the transformation matrix between two sets of scan data and parses them into TransformationComponents
    /// @param first_scan First set of scan data composed of polar coordinates
    /// @param second_scan Second set of scan data composed of polar coordinates
    /// @param current_rotation The rotation angle of the particle at the time of the first scan. 
    //                          Used to align local axis of the scan data with the global axis of the map
    /// @return TransformationComponents got from the calculated transformation matrix
    TransformationComponents computeTransformation(Eigen::MatrixX2d first_scan, Eigen::MatrixX2d second_scan, double current_rotation);

private:
    
    /// @brief Use IterativeClosestPoint to find a precise alignment from the source cloud to the target cloud
    /// @param source_points The "source" points, i.e., the points that must be transformed to align with the target point cloud
    /// @param target_points The "target" points, i.e., the points to which the source point cloud will be aligned
    /// @return A transformation matrix that will precisely align the points in source to the points in target
    Eigen::Matrix4f computeAlignment(const PointCloud::Ptr &source_points, const PointCloud::Ptr &target_points);
    /// @brief Converts the matrix to a point cloud aligned with the global x and y axis
    /// @param matrix Set of scan data composed of polar coordinates
    /// @param current_rotation The rotation angle of the particle at the time of the first scan. 
    //                          Used to align local axis of the scan data with the global axis of the map
    /// @return The PointCloud composed of all the points within the given matrix
    PointCloud matrixToPointCloud(Eigen::MatrixX2d matrix, double current_rotation);
    /// @brief Aligns polar point to the global axis and parses it into x, y and z coordinates with z always being zero
    /// @param polar_point The polar point 
    /// @param current_rotation The rotation angle of the particle at the time of the first scan. 
    /// @return Cartesian representation of the polar point (X,Y,0)
    Eigen::RowVector3f polarToCartesianXYZ(Eigen::RowVector2d polar_point, double current_rotation);
    /// @brief Extracts the TransformationComponents from the transformation matrix computed by the icp algorithm
    /// @param transformation_matrix The transformation matrix. Result of the icp algorithm
    /// @return TransformationComponents composed of a vector with the x and y delta and the rotation delta between the two sets of scan data
    TransformationComponents extractTransformationComponents(Eigen::Matrix4f transformation_matrix);
};

#endif //__PCL_HANDLER_H__
