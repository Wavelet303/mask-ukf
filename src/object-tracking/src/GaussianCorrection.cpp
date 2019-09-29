/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <GaussianCorrection.h>

#include <exception>

using namespace bfl;
using namespace Eigen;


void GaussianCorrection::correct(const GaussianMixture& pred_state, GaussianMixture& corr_state)
{
    if (!skip_)
        correctStep(pred_state, corr_state);
    else
        corr_state = pred_state;
}


bool GaussianCorrection::skip(const bool status)
{
    skip_ = status;

    return true;
}


bool GaussianCorrection::freeze_measurements(const Data& data)
{
    return getMeasurementModel().freeze(data);
}


std::pair<bool, VectorXd> GaussianCorrection::getLikelihood()
{
    throw std::runtime_error("ERROR::GAUSSIANCORRECTIONx::GETLIKELIHOOD\nERROR:\n\tCall to unimplemented base class method.");
}
