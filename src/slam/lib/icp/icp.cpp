#include <iostream>
#include <numeric>

#include "icp/icp.h"
#include "Eigen/Eigen"

Eigen::Matrix3d best_fit_transform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B)
{
    /*
    Notice:
    1/ JacobiSVD return U,S,V, S as a vector, "use U*S*Vt" to get original Matrix;
    2/ matrix type 'MatrixXd' or 'MatrixXf' matters.
    */
    Eigen::Matrix3d T = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Vector2d centroid_A(0, 0);
    Eigen::Vector2d centroid_B(0, 0);
    Eigen::MatrixXd AA = A;
    Eigen::MatrixXd BB = B;
    int row = A.rows();

    for (int i = 0; i < row; i++)
    {
        centroid_A += A.block<1, 2>(i, 0).transpose();
        centroid_B += B.block<1, 2>(i, 0).transpose();
    }
    centroid_A /= row;
    centroid_B /= row;
    for (int i = 0; i < row; i++)
    {
        AA.block<1, 2>(i, 0) = A.block<1, 2>(i, 0) - centroid_A.transpose();
        BB.block<1, 2>(i, 0) = B.block<1, 2>(i, 0) - centroid_B.transpose();
    }

    Eigen::MatrixXd H = AA.transpose() * BB;
    Eigen::MatrixXd U;
    Eigen::VectorXd S;
    Eigen::MatrixXd V;
    Eigen::MatrixXd Vt;
    Eigen::Matrix2d R;
    Eigen::Vector2d t;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    S = svd.singularValues();
    V = svd.matrixV();
    Vt = V.transpose();

    R = Vt.transpose() * U.transpose();

    if (R.determinant() < 0)
    {
        Vt.block<1, 2>(1, 0) *= -1;
        R = Vt.transpose() * U.transpose();
    }

    t = centroid_B - R * centroid_A;

    T.block<2, 2>(0, 0) = R;
    T.block<2, 1>(0, 2) = t;
    return T;
}

/*
typedef struct{
    Eigen::Matrix4d trans;
    std::vector<float> distances;
    int iter;
}  ICP_OUT;
*/

ICP_OUT icp(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, int max_iterations, double tolerance)
{
    int row = A.rows();
    Eigen::MatrixXd src = Eigen::MatrixXd::Ones(2 + 1, row);
    Eigen::MatrixXd src3d = Eigen::MatrixXd::Ones(2, row);
    Eigen::MatrixXd dst = Eigen::MatrixXd::Ones(2 + 1, row);
    NEIGHBOR neighbor;
    Eigen::Matrix3d T;
    Eigen::MatrixXd dst_chorder = Eigen::MatrixXd::Ones(2, row);
    ICP_OUT result;
    int iter = 0;

    for (int i = 0; i < row; i++)
    {
        src.block<2, 1>(0, i) = A.block<1, 2>(i, 0).transpose();
        src3d.block<2, 1>(0, i) = A.block<1, 2>(i, 0).transpose();
        dst.block<2, 1>(0, i) = B.block<1, 2>(i, 0).transpose();
    }

    double prev_error = 0;
    double mean_error = 0;
    for (int i = 0; i < max_iterations; i++)
    {
        neighbor = nearest_neighbot(src3d.transpose(), B);

        for (int j = 0; j < row; j++)
        {
            dst_chorder.block<2, 1>(0, j) = dst.block<2, 1>(0, neighbor.indices[j]);
        }

        T = best_fit_transform(src3d.transpose(), dst_chorder.transpose());

        src = T * src;
        for (int j = 0; j < row; j++)
        {
            src3d.block<2, 1>(0, j) = src.block<2, 1>(0, j);
        }

        mean_error = std::accumulate(neighbor.distances.begin(), neighbor.distances.end(), 0.0) / neighbor.distances.size();
        if (abs(prev_error - mean_error) < tolerance)
        {
            break;
        }
        prev_error = mean_error;
        iter = i + 2;
    }

    T = best_fit_transform(A, src3d.transpose());
    result.trans = T;
    result.distances = neighbor.distances;
    result.iter = iter;

    return result;
}

/*
typedef struct{
    std::vector<float> distances;
    std::vector<int> indices;
} NEIGHBOR;
*/

NEIGHBOR nearest_neighbot(const Eigen::MatrixXd &src, const Eigen::MatrixXd &dst)
{
    int row_src = src.rows();
    int row_dst = dst.rows();
    Eigen::Vector2d vec_src;
    Eigen::Vector2d vec_dst;
    NEIGHBOR neigh;
    float min = 100;
    int index = 0;
    float dist_temp = 0;

    for (int ii = 0; ii < row_src; ii++)
    {
        vec_src = src.block<1, 2>(ii, 0).transpose();
        min = 100;
        index = 0;
        dist_temp = 0;
        for (int jj = 0; jj < row_dst; jj++)
        {
            vec_dst = dst.block<1, 2>(jj, 0).transpose();
            dist_temp = dist(vec_src, vec_dst);
            if (dist_temp < min)
            {
                min = dist_temp;
                index = jj;
            }
        }
        // cout << min << " " << index << endl;
        // neigh.distances[ii] = min;
        // neigh.indices[ii] = index;
        neigh.distances.push_back(min);
        neigh.indices.push_back(index);
    }

    return neigh;
}

float dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb)
{
    return sqrt((pta[0] - ptb[0]) * (pta[0] - ptb[0]) + (pta[1] - ptb[1]) * (pta[1] - ptb[1]));
}
