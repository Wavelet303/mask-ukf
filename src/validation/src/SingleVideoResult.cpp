/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <SingleVideoResult.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;


SingleVideoResult::SingleVideoResult(const std::string& object_name, const std::string& video_name, const std::string& result_path) :
    video_name_(video_name)
{
    /* Compose path. */
    std::string root_result = result_path;
    if (root_result.back() != '/')
        root_result += '/';

    /* Compose estimate file name. */
    std::string estimate_file_name = root_result + object_name + "/" + video_name + "/object-tracking_estimate.txt";

    /* Compose ground truth file name. */
    std::string ground_truth_file_name;
    ground_truth_file_name = root_result + object_name + "/" + video_name + "/object-tracking_ground_truth.txt";

    /* Compose fps file name. */
    std::string fps_file_name = root_result + object_name + "/" + video_name + "/object-tracking_fps.txt";

    /* Load estimate. */
    bool valid_estimate = false;
    std::tie(valid_estimate, estimate_) = readDataFromFile(estimate_file_name, 14);
    if (!valid_estimate)
    {
        std::string error = log_ID_ + "::ctor. Error: cannot load results from file " + estimate_file_name;
        throw(std::runtime_error(error));
    }

    /* Load ground truth. */
    bool valid_ground_truth = false;
    std::tie(valid_ground_truth, ground_truth_) = readDataFromFile(ground_truth_file_name, 9);
    if (!valid_ground_truth)
    {
        std::string error = log_ID_ + "::ctor. Error: cannot load ground truth from file " + ground_truth_file_name;
        throw(std::runtime_error(error));
    }

    /* Load fps. */
    bool valid_fps = false;
    std::tie(valid_fps, fps_) = readDataFromFile(fps_file_name, 1);
    if (!valid_fps)
    {
        std::string error = log_ID_ + "::ctor. Error: cannot load fps from file " + fps_file_name;
        throw(std::runtime_error(error));
    }

    if ((ground_truth_.cols() - estimate_.cols()) > 1)
    {
        /* Handle validation sets having missing frames such as DenseFusion. */
        std::cout << "Padding missing frames" << std::endl;
        estimate_ = padMissingFrames(estimate_, ground_truth_.cols());

        if (ground_truth_.cols() != estimate_.cols())
        {
            std::string error = log_ID_ + "::ctor. Error: unable to pad missing frames (" + object_name + ", " + video_name + ", #GT: " + std::to_string(ground_truth_.cols()) + ", #EST: " + std::to_string(estimate_.cols()) + ")";
            throw(std::runtime_error(error));
        }
    }
}


Transform<double, 3, Affine> SingleVideoResult::getObjectPose(const std::size_t& frame_id) const
{
    // frame_id starts from 1 in YCB Video dataset
    VectorXd vector = estimate_.col(frame_id - 1);

    return makeTransform(vector.segment<7>(0));
}


Transform<double, 3, Affine> SingleVideoResult::getGroundTruthPose(const std::size_t& frame_id) const
{
    // frame_id starts from 1 in YCB Video dataset
    VectorXd vector = ground_truth_.col(frame_id - 1);

    return makeTransform(vector.segment<7>(2));
}


Eigen::VectorXd SingleVideoResult::getObjectVelocity(const std::size_t& frame_id) const
{
    VectorXd vector = estimate_.col(frame_id -1);

    Transform<double, 3, Affine> pose = getObjectPose(frame_id);
    Vector3d euler_angles = pose.rotation().eulerAngles(2, 1, 0);
    double rot_z = euler_angles(0);
    double rot_y = euler_angles(1);
    MatrixXd T1 = AngleAxisd(rot_z, Vector3d::UnitZ()).toRotationMatrix();
    MatrixXd T2 = T1 * AngleAxisd(rot_y, Vector3d::UnitY()).toRotationMatrix();

    // Assign the linear velocity and Euler angular rates
    VectorXd vel = vector.segment<6>(7);

    Vector3d w_1 = Vector3d::Zero();
    w_1(2) = vel(3);

    Vector3d w_2 = T1.col(1) * vel(4);

    Vector3d w_3 = T2.col(0) * vel(5);

    // Express the Euler angular rates in terms of plain angular velocity
    vel.tail<3>() = w_1 + w_2 + w_3;

    return vel;
}


Eigen::VectorXd SingleVideoResult::getGroundTruthVelocity(const std::size_t& frame_id, const std::size_t& frame_id_0)
{
    std::size_t frame = frame_id - 1;
    std::size_t frame_0 = frame_id_0 - 1;

    if (frame == frame_0)
    {
        last_v_ = VectorXd::Zero(6);
        return VectorXd::Zero(6);
    }

    VectorXd vector = ground_truth_.col(frame).segment<7>(2);
    VectorXd vector_prev = ground_truth_.col(frame - 1).segment<7>(2);

    VectorXd fin_diff(6);
    // Evaluate linear velocity using finite differences assuming 30 fps
    fin_diff.head<3>() = (vector.head<3>() - vector_prev.head<3>()) / (1/30.0);

    // Evaluate angular velocity using finite differences between rotation matrices
    // of consecutive frames assuming 30 fps
    Transform<double, 3, Affine> pose = getObjectPose(frame_id);
    Transform<double, 3, Affine> pose_prev = getObjectPose(frame_id - 1);
    Matrix3d delta_rot = pose.rotation() * pose_prev.rotation().transpose();
    double theta = std::acos((delta_rot.trace() -1) / 2.0) + std::numeric_limits<double>::min();
    Matrix3d w_tensor = 1 / (2 * (1 / 30.0)) * theta / std::sin(theta) * (delta_rot - delta_rot.transpose());

    Vector3d w;
    w(0) = - w_tensor(1, 2);
    w(1) = w_tensor(0, 2);
    w(2) = - w_tensor(0, 1);
    fin_diff.tail<3>() = w;

    // Exponential moving averaged ground truth to avoid spikes
    double alpha = 0.3;
    VectorXd new_v = last_v_ + alpha * (fin_diff - last_v_);
    last_v_ = new_v;

    return new_v;
}


double SingleVideoResult::getFPS(const std::size_t& frame_id) const
{
    return fps_.col(frame_id -1)(0);
}


std::size_t SingleVideoResult::getNumberFrames() const
{
    return ground_truth_.cols();
}


std::string SingleVideoResult::getVideoName() const
{
    return video_name_;
}


std::pair<bool, MatrixXd> SingleVideoResult::readDataFromFile(const std::string& filename, const std::size_t num_fields)
{
    MatrixXd data;

    std::ifstream istrm(filename);

    if (!istrm.is_open())
    {
        std::cout << log_ID_ + "::readDataFromFile. Error: failed to open " << filename << '\n';

        istrm.close();

        return std::make_pair(false, MatrixXd(0,0));
    }

    std::vector<std::string> istrm_strings;
    std::string line;
    while (std::getline(istrm, line))
    {
        istrm_strings.push_back(line);
    }

    data.resize(num_fields, istrm_strings.size());
    std::size_t found_lines = 0;
    for (auto line : istrm_strings)
    {
        std::size_t found_fields = 0;
        std::string number_str;
        std::istringstream iss(line);

        while (iss >> number_str)
        {
            std::size_t index = (num_fields * found_lines) + found_fields;
            *(data.data() + index) = std::stod(number_str);
            found_fields++;
        }
        if (num_fields != found_fields)
        {
            std::cout << log_ID_ + "::readDataFromFile. Error: malformed input file " << filename << '\n';

            return std::make_pair(false, MatrixXd(0,0));
        }
        found_lines++;
    }

    istrm.close();

    return std::make_pair(true, data);
}


Eigen::MatrixXd SingleVideoResult::padMissingFrames(const Eigen::MatrixXd& data, const std::size_t& expected_length)
{
    MatrixXd padded_data(data.rows(), expected_length);
    std::size_t index_key = data.rows() - 1;

    for (std::size_t i = 0, j = 0; i < padded_data.cols(); i++)
    {
        if (data.col(j)(index_key) == (i + 1))
        {
            padded_data.col(i) = data.col(j);
            j++;
        }
        else
        {
            // DenseFusion might have missing frames due to missing detection in PoseCNN
            // When missing, they are identified as having an infinite error as per the code
            // that can be found here https://github.com/j96w/DenseFusion/blob/master/replace_ycb_toolbox/evaluate_poses_keyframe.m
            VectorXd padded_col = VectorXd::Ones(data.rows()) * std::numeric_limits<double>::infinity();
            padded_col(index_key) = i;
            padded_data.col(i) = padded_col;
        }
    }

    return padded_data;
}


Transform<double, 3, Affine> SingleVideoResult::makeTransform(const VectorXd pose) const
{
    /**
     * pose expected to be a 7-dimensional vector containing
     * x - y - z - axis vector - angle
     */

    Transform<double, 3, Affine> transform;

    /* Compose translational part. */
    transform = Translation<double, 3>(pose.head<3>());

    /* Compose rotational part. */
    AngleAxisd rotation(pose(6), pose.segment<3>(3));
    transform.rotate(rotation);

    return transform;
}
