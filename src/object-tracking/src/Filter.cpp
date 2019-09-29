/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifdef _OPENMP
#include <omp.h>
#endif

#include <Filter.h>

#include <BayesFilters/utils.h>

#include <yarp/eigen/Eigen.h>
#include <yarp/os/LogStream.h>

// using namespace bfl;
using namespace Eigen;
using namespace yarp::eigen;
using namespace yarp::os;
using namespace yarp::sig;


Filter::Filter
(
    const std::string port_prefix,
    std::unique_ptr<InitGroundTruth> initialization,
    std::unique_ptr<bfl::GaussianPrediction> prediction,
    std::unique_ptr<GaussianCorrection> correction,
    std::unique_ptr<Validator2D> validator
) :
    GaussianFilter(std::move(prediction), std::move(correction)),
    initialization_(std::move(initialization)),
    validator_(std::move(validator))
{
    // Open data output port
    if (!port_data_out_.open("/" + port_prefix + "/estimate:o"))
    {
        std::string err = "FILTER::CTOR::ERROR\n\tError: cannot estimate output port.";
        throw(std::runtime_error(err));
    }

    // Open ground truth input port
    if (!port_ground_truth_in_.open("/" + port_prefix + "/ground-truth:i"))
    {
        std::string err = "FILTER::CTOR::ERROR\n\tError: cannot ground truth input port.";
        throw(std::runtime_error(err));
    }

    // Open RPC input port for commands
    if (!port_rpc_command_.open("/" + port_prefix + "/cmd:i"))
    {
        std::string err = "FILTER::CTOR::ERROR\n\tError: cannot open RPC command port.";
        throw(std::runtime_error(err));
    }

    if (!(this->yarp().attachAsServer(port_rpc_command_)))
    {
        std::string err = "FILTER::CTOR::ERROR\n\tError: cannot attach the RPC command port.";
        throw(std::runtime_error(err));
    }

    // Placeholder for ground truth
    last_ground_truth_ = VectorXd::Ones(9) * (-1);
}


Filter::~Filter()
{
    disable_log();
    port_data_out_.close();
}


bool Filter::run_filter()
{
    run();
    return true;
}


bool Filter::reset_filter()
{
    reset();

    // Reset the kinematic model
    prediction().getStateModel().setProperty("reset");

    // Reset the measurement model
    correction().getMeasurementModel().setProperty("reset");

    // Reset the validator
    validator_->reset();

    keyframe_counter_ = 1;

    return true;
}


bool Filter::stop_filter()
{
    reboot();

    // Reset the kinematic model
    prediction().getStateModel().setProperty("reset");

    // Reset the measurement model
    correction().getMeasurementModel().setProperty("reset");

    // Reset the validator
    validator_->reset();

    keyframe_counter_ = 1;

    return true;
}


void Filter::pause_filter()
{
    pause_ = true;
}


void Filter::resume_filter()
{
    pause_ = false;
}


bool Filter::skip_step(const std::string& what_step, const bool status)
{
    return skip(what_step, status);
}


bool Filter::disable_logs()
{
    disable_log();
    correction().getMeasurementModel().setProperty("disable_log");

    return true;
}


bool Filter::quit()
{
    return teardown();
}


bool Filter::runCondition()
{
    return true;
}


bool Filter::initialization()
{
    pred_belief_.resize(12);
    corr_belief_.resize(12);

    pred_belief_.mean() = initialization_->getInitialPose();
    pred_belief_.covariance() = initialization_->getInitialCovariance();

    return true;
}


void Filter::skipFrames(std::size_t number_frames)
{
    skip_frames_ = true;
    skip_frames_number_ = number_frames;
}


std::vector<std::string> Filter::log_file_names(const std::string& prefix_path, const std::string& prefix_name)
{
    return  {prefix_path + "/" + prefix_name + "_estimate",
             prefix_path + "/" + prefix_name + "_ground_truth",
             prefix_path + "/" + prefix_name + "_fps"};
}


void Filter::filteringStep()
{
    // In case the user pauses the filtering recursion
    if (pause_)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return;
    }

    // Used for automatized validation in the non-real-time scenario
    // When there are no more images in the dataset, quit() is invoked
    if(!correction().getMeasurementModel().setProperty("measurements_available"))
    {
        quit();
        return;
    }

    startTimeCount();

    // Retrieve ground truth for synchronized logging purposes
    // Used in the real-time scenario
    Vector* gt = port_ground_truth_in_.read(false);
    if (gt != nullptr)
    {
        // FIXME: this is for backward compatibility and need to be removed
        last_ground_truth_(0) = 0;
        last_ground_truth_(1) = 0;
        //
        last_ground_truth_.segment<7>(2) = toEigen(*gt);
    }

    if ((getFilteringStep() == 0 || skip_frames_initialization_))
        correction().freeze_measurements(pred_belief_);
    else
        correction().freeze_measurements(corr_belief_);

    if ((getFilteringStep() != 0) && (!skip_frames_initialization_))
        prediction().predict(corr_belief_, pred_belief_);
    correction().correct(pred_belief_, corr_belief_);

    if (skip_frames_initialization_)
        skip_frames_initialization_ = false;

    // Render 2d evaluation using current estimate
    Transform<double, 3, Affine> object_transform;
    object_transform = Translation<double, 3>(corr_belief_.mean().head<3>());
    AngleAxisd rotation = AngleAxisd(AngleAxisd(corr_belief_.mean(9), Vector3d::UnitZ()) *
                          AngleAxisd(corr_belief_.mean(10), Vector3d::UnitY()) *
                          AngleAxisd(corr_belief_.mean(11), Vector3d::UnitX()));
    object_transform.rotate(rotation);
    validator_->renderEvaluation(object_transform);

    // Tick the state model
    prediction().getStateModel().setProperty("tick");

    // Evaluate fps
    double execution_time = stopTimeCount();
    double fps = (execution_time != 0) ? (1 / execution_time) : -1;

    // Send data
    VectorXd estimate(14);
    estimate.head<3>() = corr_belief_.mean().head<3>();
    AngleAxisd angle_axis(AngleAxisd(corr_belief_.mean(9), Vector3d::UnitZ()) *
                          AngleAxisd(corr_belief_.mean(10), Vector3d::UnitY()) *
                          AngleAxisd(corr_belief_.mean(11), Vector3d::UnitX()));
    estimate.segment<3>(3) = angle_axis.axis();
    estimate(6) = angle_axis.angle();
    estimate.segment<3>(7) = corr_belief_.mean().segment<3>(3);
    estimate.segment<3>(10) = corr_belief_.mean().segment<3>(6);
    estimate(13) = keyframe_counter_;

    /**
     * As explained in the paper, we initialize the filter from the first frame or
     * after the object was heavily occlued. In practice, the second approach is employed
     * only in case (i.e. video sequence 0059, object 010_potted_meat_can).
     * See main.cpp.
     */
    if (skip_frames_)
    {
        if(keyframe_counter_ < skip_frames_number_)
            initialization_->step();
        else
        {
            pred_belief_.mean() = initialization_->getInitialPose();
            pred_belief_.covariance() = initialization_->getInitialCovariance();
            skip_frames_ = false;
            skip_frames_initialization_ = true;
        }
    }
    else
        logger(estimate.transpose(), last_ground_truth_.transpose(), fps);

    // Increase keyframe counter
    keyframe_counter_++;

    Vector& estimate_yarp = port_data_out_.prepare();
    estimate_yarp.resize(14);
    toEigen(estimate_yarp) = estimate;
    port_data_out_.write();
    // std::cout << fps << " fps" << std::endl;
}


void Filter::startTimeCount()
{
#ifdef _OPENMP
    omp_time_0_ = omp_get_wtime();
#else
    std_time_0_ = std::chrono::high_resolution_clock::now();
#endif
}


double Filter::stopTimeCount()
{
#ifdef _OPENMP
    return omp_get_wtime() - omp_time_0_;
#else
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - std_time_0_).count() / 1000.0;
#endif
}
