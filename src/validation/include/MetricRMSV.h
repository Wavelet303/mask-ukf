/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef METRICRMSV_H
#define METRICRMSV_H

#include <Eigen/Dense>

#include <Metric.h>


class MetricRMSV : public Metric
{
public:
    MetricRMSV();

    MetricRMSV(const Eigen::VectorXd& symmetry_axis);

    ~MetricRMSV();

    Eigen::VectorXd evaluate(const Eigen::Transform<double, 3, Eigen::Affine>& estimate, const Eigen::Transform<double, 3, Eigen::Affine>& ground_truth, const Eigen::MatrixXd& model) override;

    Eigen::VectorXd evaluate(const Eigen::VectorXd& estimate, const Eigen::VectorXd& ground_truth, const Eigen::Transform<double, 3, Eigen::Affine>& ground_truth_pose) override;

private:
    Eigen::VectorXd symmetry_axis_;

    bool has_symmetry_ = false;
};

#endif /* METRICRMSV_H */
