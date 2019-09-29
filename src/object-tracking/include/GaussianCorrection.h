/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef GAUSSIANCORRECTION__H
#define GAUSSIANCORRECTION__H

#include <BayesFilters/GaussianMixture.h>
#include <BayesFilters/MeasurementModel.h>
#include <BayesFilters/any.h>

#include <Eigen/Dense>

class GaussianCorrection
{
public:
    virtual ~GaussianCorrection() noexcept = default;

    void correct(const bfl::GaussianMixture& pred_state, bfl::GaussianMixture& corr_state);

    bool skip(const bool status);

    virtual bfl::MeasurementModel& getMeasurementModel() = 0;

    bool freeze_measurements(const bfl::Data& data = bfl::Data());

    virtual std::pair<bool, Eigen::VectorXd> getLikelihood();


protected:
    GaussianCorrection() noexcept = default;

    GaussianCorrection(const GaussianCorrection& correction) noexcept = delete;

    GaussianCorrection& operator=(const GaussianCorrection& correction) noexcept = delete;

    GaussianCorrection(GaussianCorrection&& correction) noexcept = default;

    GaussianCorrection& operator=(GaussianCorrection&& correction) noexcept = default;

    virtual void correctStep(const bfl::GaussianMixture& pred_state, bfl::GaussianMixture& corr_state) = 0;

private:
    bool skip_ = false;
};

#endif /* GAUSSIANCORRECTION__H */
