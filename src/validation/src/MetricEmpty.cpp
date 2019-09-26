/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <MetricEmpty.h>

using namespace Eigen;


MetricEmpty::MetricEmpty()
{ }


MetricEmpty::~MetricEmpty()
{ }


VectorXd MetricEmpty::evaluate(const Eigen::Transform<double, 3, Eigen::Affine>& estimate, const Eigen::Transform<double, 3, Eigen::Affine>& ground_truth, const Eigen::MatrixXd& model)
{
    return VectorXd();
}
