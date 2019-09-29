/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef MASKSEGMENTATION_H
#define MASKSEGMENTATION_H

#include <Eigen/Dense>

#include <Camera.h>
#include <PointCloudSegmentation.h>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Image.h>


class MaskSegmentation : public PointCloudSegmentation
{
public:

    MaskSegmentation(const std::string& port_prefix, const std::string& mask_name, const std::size_t& depth_stride, const bool& handle_mask_streaming);

    MaskSegmentation(const std::string& port_prefix, const std::string& path, const std::string& mask_name, const std::size_t& depth_stride, const std::string& masks_set);

    ~MaskSegmentation();

    bool freezeSegmentation(Camera& camera) override;

    std::tuple<bool, std::size_t, Eigen::MatrixXd> extractPointCloud(Camera& camera, const Eigen::Ref<const Eigen::MatrixXf>& depth, const double& max_depth) override;

    bool setProperty(const std::string& property) override;

protected:
    std::string composeFileName(const std::size_t& index, const std::size_t& number_digits);

    bool enableMaskStreaming();

    std::tuple<bool, bool, cv::Mat> getMask(Camera& camera);

    void drawMaskOnCamera(const cv::Mat& mask_points, Camera& camera);

    std::size_t depth_stride_;

    Eigen::Transform<double, 3, Eigen::Affine> camera_pose_;

    std::string mask_name_;

    std::vector<std::pair<int, int>> coordinates_;

    cv::Mat mask_;

    cv::Mat image_mask_;

    bool mask_initialized_ = false;

    bool mask_streaming_initialized_ = false;

    bool offline_ = false;

    const bool handle_mask_streaming_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_image_out_;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono>> port_image_in_;

    yarp::os::RpcClient mask_rpc_client_;

    /**
     * Required for offline execution.
     */

    std::string path_mask_images_;

    std::size_t number_of_digits_ = 6;

    int head_ = 1;

    const std::string log_ID_ = "MaskSegmentation";
};

#endif /* MASKSEGMENTATION_H */
