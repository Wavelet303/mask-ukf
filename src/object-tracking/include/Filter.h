/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef FILTER_H
#define FILTER_H

#include <BayesFilters/Gaussian.h>
#include <BayesFilters/GaussianPrediction.h>
#include <BayesFilters/MeasurementModel.h>

#include <GaussianCorrection.h>
#include <GaussianFilter.h>
#include <InitGroundTruth.h>
#include <ObjectMeasurements.h>
#include <Validator2D.h>

#include <chrono>
#include <memory>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Vector.h>

#include <thrift/ObjectTrackingIDL.h>


class Filter : public GaussianFilter,
               public ObjectTrackingIDL
{
public:
    Filter
    (
        const std::string port_prefix,
        std::unique_ptr<InitGroundTruth> initialization,
        std::unique_ptr<bfl::GaussianPrediction> prediction,
        std::unique_ptr<GaussianCorrection> correction,
        std::unique_ptr<Validator2D> validator
    );

    virtual ~Filter();

    bool run_filter() override;

    bool reset_filter() override;

    bool stop_filter() override;

    void pause_filter() override;

    void resume_filter() override;

    bool skip_step(const std::string& what_step, const bool status) override;

    bool disable_logs() override;

    bool quit() override;

    bool runCondition() override;

    bool initialization() override;

    void skipFrames(std::size_t number_frames);

protected:
    std::vector<std::string> log_file_names(const std::string& prefix_path, const std::string& prefix_name) override;

    void filteringStep() override;

private:
    void startTimeCount();

    double stopTimeCount();

    double omp_time_0_;

    std::chrono::high_resolution_clock::time_point std_time_0_;

    bfl::Gaussian pred_belief_;

    bfl::Gaussian corr_belief_;

    Eigen::VectorXd last_ground_truth_;

    std::unique_ptr<InitGroundTruth> initialization_;

    std::unique_ptr<Validator2D> validator_;

    std::size_t keyframe_counter_ = 1;

    yarp::os::BufferedPort<yarp::sig::Vector> port_data_out_;

    yarp::os::BufferedPort<yarp::sig::Vector> port_ground_truth_in_;

    yarp::os::Port port_rpc_command_;

    bool pause_ = false;

    /* For non real time execution. */
    bool skip_frames_ = false;

    std::size_t skip_frames_number_ = 0;

    bool skip_frames_initialization_ = false;

    const std::string log_ID_ = "Filter";
};

#endif /* FILTER_H */
