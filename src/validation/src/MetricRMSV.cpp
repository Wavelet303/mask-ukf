/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <MetricRMSV.h>
#include <iostream>

using namespace Eigen;


MetricRMSV::MetricRMSV()
{ }


MetricRMSV::MetricRMSV(const Eigen::VectorXd& symmetry_axis) :
    symmetry_axis_(symmetry_axis),
    has_symmetry_(true)
{ }


MetricRMSV::~MetricRMSV()
{ }


VectorXd MetricRMSV::evaluate(const Eigen::VectorXd& estimate, const Eigen::VectorXd& ground_truth, const Eigen::Transform<double, 3, Eigen::Affine>& ground_truth_pose)
{
    VectorXd errors(2);

    // Identify if there were skipped frames
    if (((estimate(0) == std::numeric_limits<double>::infinity()) &&
         (estimate(1) == std::numeric_limits<double>::infinity()) &&
         (estimate(2) == std::numeric_limits<double>::infinity())))
    {
        // Warning: this check is required since some DenseFusion frames might be missing.
        // When missing, they are identified as having an infinite estimate
        // In such cases, we decided to assign a null error for the evaluation of the RMSE
        errors(0) = 0.0;
        errors(1) = 0.0;

        return errors;
    }

    errors(0) = (estimate.head<3>() - ground_truth.head<3>()).squaredNorm();

    // In case of symmetry along an axis, we remove part of the angular velocity projected to that axis.
    if (has_symmetry_)
    {
        Vector3d axis_world = ground_truth_pose.rotation() * symmetry_axis_;
        Vector3d projected_w = estimate.tail<3>().dot(axis_world) * axis_world;
        Vector3d rest_w = estimate.tail<3>() - projected_w;
        errors(1) = (rest_w - ground_truth.tail<3>()).squaredNorm();
    }
    else
        errors(1) = (estimate.tail<3>() - ground_truth.tail<3>()).squaredNorm();

    return errors;
}


VectorXd MetricRMSV::evaluate(const Eigen::Transform<double, 3, Eigen::Affine>& estimate, const Eigen::Transform<double, 3, Eigen::Affine>& ground_truth, const Eigen::MatrixXd& model)
{
    // This method is not implemented and should not be used
    VectorXd errors(2);

    return errors;
}
