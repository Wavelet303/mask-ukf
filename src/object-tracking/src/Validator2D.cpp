/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifdef _OPENMP
#include <omp.h>
#endif

#include <Validator2D.h>
#include <CameraParameters.h>
#include <RealsenseCamera.h>
#include <YCBVideoCamera.h>
#include <YCBVideoCameraNRT.h>

#include <yarp/cv/Cv.h>
#include <yarp/sig/Image.h>


using namespace Eigen;
using namespace yarp::cv;
using namespace yarp::os;
using namespace yarp::sig;


Validator2D::Validator2D
(
    std::shared_ptr<PointCloudPrediction> point_cloud_prediction,
    const std::string& port_prefix,
    const std::string& camera_name,
    const std::string& camera_fallback_key
) :
    prediction_(point_cloud_prediction)
{
    if (!port_image_in_.open("/" + port_prefix + "/validationImage:i"))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open background input port.";
        throw(std::runtime_error(err));
    }

    if (!port_image_out_.open("/" + port_prefix + "/validationImage:o"))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open output image port.";
        throw(std::runtime_error(err));
    }

    // Initialize camera required to extract depth
    if (camera_name == "RealsenseCamera")
    {
        camera_ = std::unique_ptr<RealsenseCamera>(new RealsenseCamera(port_prefix, "object-tracking", camera_fallback_key));
    }
    else if (camera_name == "YCBVideoCamera")
    {
        camera_ = std::unique_ptr<YcbVideoCamera>(new YcbVideoCamera(port_prefix, "object-tracking", camera_fallback_key));
    }
    else
    {
        std::string err = log_ID_ + "::ctor. The requested camera is not available. Requested camera is " + camera_name;
        throw(std::runtime_error(err));
    }
    camera_->initialize();
}


Validator2D::Validator2D
(
    std::shared_ptr<PointCloudPrediction> point_cloud_prediction,
    const std::string& port_prefix,
    const std::string& camera_path
) :
    prediction_(point_cloud_prediction)
{
    if (!port_image_in_.open("/" + port_prefix + "/validationImage:i"))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open background input port.";
        throw(std::runtime_error(err));
    }

    if (!port_image_out_.open("/" + port_prefix + "/validationImage:o"))
    {
        std::string err = log_ID_ + "::ctor. Error: cannot open output image port.";
        throw(std::runtime_error(err));
    }

    // Initialize camera required to extract depth
    camera_ = std::unique_ptr<YcbVideoCameraNrt>(new YcbVideoCameraNrt(camera_path, 320, 240, "object-tracking"));
    camera_->initialize();

    // Connect to a default yarpviewer
    Network::connect("/" + port_prefix + "/validationImage:o", "/object-tracking/viewer/validator:i", "tcp");
}


Validator2D::~Validator2D()
{
    port_image_in_.close();
    port_image_out_.close();
}


bool Validator2D::reset()
{
    camera_->reset();

    return true;
}


bool Validator2D::renderEvaluation(const Transform<double, 3, Affine> object_pose)
{
    // Freeze camera
    camera_->freeze();

    // Get rgb image
    bool valid_image;
    std::tie(valid_image, image_in_) = camera_->getRgbImage(false);
    if (!valid_image)
        return false;

    // Express pose with respect to the camera reference frame
    bool valid_pose = false;
    Transform<double, 3, Affine> camera_pose;

    std::tie(valid_pose, camera_pose) = camera_->getCameraPose(false);
    if (!valid_pose)
        return false;

    Transform<double, 3, Affine> cam_to_object = camera_pose.inverse() * object_pose;

    // Get the model virtual point cloud from class PointCloudPrediction
    MatrixXd cloud = prediction_->evaluateModel(cam_to_object);

    // Get camera intrinsic parameters
    bool valid_camera_parameters;
    CameraParameters camera_parameters;
    std::tie(valid_camera_parameters, camera_parameters) = camera_->getIntrinsicParameters();
    if (!valid_camera_parameters)
        return false;

    // Transform to a cv contour, i.e. a std::vector of std::vector of cv::Point
    // std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> contour(cloud.cols());

#pragma omp parallel for
    for (std::size_t i = 0; i < cloud.cols(); i++)
    {
        int u = camera_parameters.cx + cloud.col(i)(0) / cloud.col(i)(2) * camera_parameters.fx;
        int v = camera_parameters.cy + cloud.col(i)(1) / cloud.col(i)(2) * camera_parameters.fy;

        contour.at(i) = cv::Point(u, v);
    }

    // Evaluate convex hull
    std::vector<std::vector<cv::Point>> hull(1);
    cv::convexHull(contour, hull[0]);

    // Superimpose on input image
    cv::drawContours(image_in_, hull, 0, cv::Scalar(0, 0, 255), 3);

    // Send validation image
    ImageOf<PixelRgb>& image_out = port_image_out_.prepare();
    image_out = fromCvMat<PixelRgb>(image_in_);
    port_image_out_.write();

    return true;
}
