/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef GAUSSIANFILTER__H
#define GAUSSIANFILTER__H

#include <BayesFilters/FilteringAlgorithm.h>
#include <BayesFilters/Gaussian.h>
#include <BayesFilters/GaussianPrediction.h>
#include <GaussianCorrection.h>


class GaussianFilter : public bfl::FilteringAlgorithm
{
public:
    virtual ~GaussianFilter() noexcept = default;

    bool skip(const std::string& what_step, const bool status) override;


protected:
    GaussianFilter(std::unique_ptr<bfl::GaussianPrediction> prediction, std::unique_ptr<GaussianCorrection> correction) noexcept;

    GaussianFilter(const GaussianFilter& filter) noexcept = delete;

    GaussianFilter& operator=(const GaussianFilter& filter) noexcept = delete;

    GaussianFilter(GaussianFilter&& filter) noexcept = delete;

    GaussianFilter& operator=(GaussianFilter&& filter) noexcept = delete;

    bfl::GaussianPrediction& prediction();

    GaussianCorrection& correction();


private:
    std::unique_ptr<bfl::GaussianPrediction> prediction_;

    std::unique_ptr<GaussianCorrection> correction_;
};

#endif /* GAUSSIANFILTER__H */
