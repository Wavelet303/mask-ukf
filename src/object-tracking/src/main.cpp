/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#include <Camera.h>
#include <Correction.h>
#include <CorrectionICP.h>
#include <DiscretizedKinematicModel.h>
#include <Filter.h>
#include <GaussianCorrection.h>
#include <GaussianFilter.h>
#include <InitGroundTruth.h>
#include <MaskSegmentation.h>
#include <ObjectPointCloudPrediction.h>
#include <PointCloudSegmentation.h>
#include <RealsenseCamera.h>
#include <StaticPrediction.h>
#include <Validator2D.h>
#include <YCBVideoCamera.h>
#include <YCBVideoCameraNRT.h>

#include <BayesFilters/AdditiveMeasurementModel.h>
#include <BayesFilters/FilteringAlgorithm.h>
#include <BayesFilters/GaussianPrediction.h>
#include <BayesFilters/KFPrediction.h>

#include <Eigen/Dense>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

#include <cstdlib>
#include <string>
#include <sstream>

using namespace bfl;
using namespace Eigen;
using namespace yarp::os;


VectorXd loadVectorDouble(Bottle &rf, const std::string key, const std::size_t size);


std::vector<std::string> loadListString(Bottle &rf, const std::string key);


std::string eigenToString(const Ref<const VectorXd>& v);


int main(int argc, char** argv)
{
    const std::string log_ID = "[Main]";
    yInfo() << log_ID << "Configuring and starting module...";

    const std::string port_prefix = "object-tracking";

    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext("object-tracking");
    rf.configure(argc, argv);

    /* Get algorithm name. */
    const std::string algorithm = rf.check("algorithm", Value("UKF")).asString();

    /* Get autostart flag. */
    const bool autostart = rf.check("autostart", Value(false)).asBool();

    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << log_ID << "YARP seems unavailable!";
        return EXIT_FAILURE;
    }

    /* Get initial conditions. */
    Bottle rf_initial_conditions = rf.findGroup("INITIAL_CONDITION");
    VectorXd cov_x_0             = loadVectorDouble(rf_initial_conditions, "cov_x_0",       3);
    VectorXd cov_v_0             = loadVectorDouble(rf_initial_conditions, "cov_v_0",       3);
    VectorXd cov_eul_0           = loadVectorDouble(rf_initial_conditions, "cov_eul_0",     3);
    VectorXd cov_eul_dot_0       = loadVectorDouble(rf_initial_conditions, "cov_eul_dot_0", 3);

    /* Camera parameters. */
    Bottle rf_camera = rf.findGroup("CAMERA");
    const std::string camera_name         = rf_camera.check("name", Value("iCubCamera")).asString();
    const std::string camera_fallback_key = rf_camera.check("fallback_key", Value("icub_320_240")).asString();

    /* Kinematic model. */
    Bottle rf_kinematic_model = rf.findGroup("KINEMATIC_MODEL");
    VectorXd kin_q_x             = loadVectorDouble(rf_kinematic_model, "q_x", 3);
    VectorXd kin_q_eul           = loadVectorDouble(rf_kinematic_model, "q_eul", 3);
    const double kin_rate        = rf_kinematic_model.check("rate", Value(30.0)).asDouble();
    const bool kin_est_period    = rf_kinematic_model.check("estimate_period", Value(true)).asBool();

    /* Measurement model. */
    Bottle rf_measurement_model = rf.findGroup("MEASUREMENT_MODEL");
    VectorXd visual_covariance           = loadVectorDouble(rf_measurement_model, "visual_covariance", 3);
    MatrixXd visual_covariance_diagonal  = visual_covariance.asDiagonal();

    /* Unscented transform. */
    Bottle rf_unscented_transform = rf.findGroup("UNSCENTED_TRANSFORM");
    const double ut_alpha = rf_unscented_transform.check("alpha", Value("1.0")).asDouble();
    const double ut_beta  = rf_unscented_transform.check("beta", Value("2.0")).asDouble();
    const double ut_kappa = rf_unscented_transform.check("kappa", Value("0.0")).asDouble();

    /* Point cloud filtering. */
    Bottle rf_point_cloud_filtering = rf.findGroup("POINT_CLOUD_FILTERING");
    const bool pc_outlier_rejection = rf_point_cloud_filtering.check("outlier_rejection", Value(false)).asBool();

    /* Depth. */
    Bottle rf_depth = rf.findGroup("DEPTH");
    const std::string depth_fetch_mode = rf_depth.check("fetch_mode", Value("new_image")).toString();
    const std::size_t depth_stride     = rf_depth.check("stride", Value(1)).asInt();

    /* Mesh parameters. */
    Bottle rf_object = rf.findGroup("OBJECT");
    const std::string object_name      = rf_object.check("object_name", Value("002_master_chef_can")).asString();
    const std::string object_point_cloud_path = rf.findPath("models/" + object_name);
    const std::string object_data_path        = rf_object.check("path", Value("null")).asString();

    /* Segmentation parameters. */
    Bottle rf_segmentation    = rf.findGroup("SEGMENTATION");
    const std::string segmentation_type = rf_segmentation.check("type", Value("in_hand")).asString();
    const std::string segmentation_set  = rf_segmentation.check("masks_set", Value("gt")).asString();
    bool handle_mask_rpc = rf_segmentation.check("handle_mask_rpc", Value(true)).asBool();

    /* Logging parameters. */
    Bottle rf_logging = rf.findGroup("LOG");
    bool enable_log = rf_logging.check("enable_log", Value(false)).asBool();
    const std::string log_path = rf_logging.check("absolute_log_path", Value("")).asString();
    if (enable_log && log_path == "")
    {
        yWarning() << "Invalid log path. Disabling log...";
        enable_log = false;
    }

    /* Log parameters. */

    yInfo() << log_ID << "Algorithm:" << algorithm;

    yInfo() << log_ID << "Initial conditions:";
    yInfo() << log_ID << "- cov_x_0: "          << eigenToString(cov_x_0);
    yInfo() << log_ID << "- cov_v_0: "          << eigenToString(cov_v_0);
    yInfo() << log_ID << "- cov_eul_0: "        << eigenToString(cov_eul_0);
    yInfo() << log_ID << "- cov_eul_dot_0: "    << eigenToString(cov_eul_dot_0);

    yInfo() << log_ID << "Camera:";
    yInfo() << log_ID << "- name:"         << camera_name;
    yInfo() << log_ID << "- fallback_key:" << camera_fallback_key;

    yInfo() << log_ID << "Kinematic model:";
    yInfo() << log_ID << "- q_x:"             << eigenToString(kin_q_x);
    yInfo() << log_ID << "- q_eul:"           << eigenToString(kin_q_eul);
    yInfo() << log_ID << "- rate:"            << kin_rate;
    yInfo() << log_ID << "- estimate_period:" << kin_est_period;

    yInfo() << log_ID << "Measurement model:";
    yInfo() << log_ID << "- visual_covariance:" << eigenToString(visual_covariance);

    yInfo() << log_ID << "Unscented transform:";
    yInfo() << log_ID << "- alpha:" << ut_alpha;
    yInfo() << log_ID << "- beta:"  << ut_beta;
    yInfo() << log_ID << "- kappa:" << ut_kappa;

    yInfo() << log_ID << "Point cloud filtering:";
    yInfo() << log_ID << "- outlier_rejection:" << pc_outlier_rejection;

    yInfo() << log_ID << "Depth:";
    yInfo() << log_ID << "- fetch_mode:" << depth_fetch_mode;
    yInfo() << log_ID << "- stride:" << depth_stride;

    yInfo() << log_ID << "Object:";
    yInfo() << log_ID << "- object_name:"         << object_name;
    yInfo() << log_ID << "- object_data_path:"    << object_data_path;
    yInfo() << log_ID << "- point cloud path is:" << object_point_cloud_path;

    yInfo() << log_ID << "Segmentation:";
    yInfo() << log_ID << "- type:" << segmentation_type;
    yInfo() << log_ID << "- handle_mask_rpc:" << handle_mask_rpc;
    yInfo() << log_ID << "- masks_set:" << segmentation_set;

    yInfo() << log_ID << "Logging:";
    yInfo() << log_ID << "- enable_log:"        << enable_log;
    yInfo() << log_ID << "- absolute_log_path:" << log_path;

    /**
     * Initialize camera
     */
    std::unique_ptr<Camera> camera;
    if (camera_name == "RealsenseCamera")
    {
        camera = std::unique_ptr<RealsenseCamera>
        (
            new RealsenseCamera(port_prefix, "object-tracking", camera_fallback_key)
        );
    }
    else if (camera_name == "YCBVideoCamera")
    {
        camera = std::unique_ptr<YcbVideoCamera>
        (
            new YcbVideoCamera(port_prefix, "object-tracking", camera_fallback_key)
        );
    }
    else if (camera_name == "YCBVideoCameraNRT")
    {
        camera = std::unique_ptr<YcbVideoCameraNrt>
        (
            new YcbVideoCameraNrt(object_data_path, 320, 240, "object-tracking")
        );
    }
    else
    {
        yError() << log_ID << "The requested camera is not available. Requested camera is" << camera_name;
        std::exit(EXIT_FAILURE);
    }
    camera->initialize();

    /**
     * Initialize object segmentation.
     */
    std::unique_ptr<PointCloudSegmentation> segmentation;

    // Masks from a stream of mask images
    if (segmentation_type == "mask")
    {
        segmentation = std::unique_ptr<MaskSegmentation>
        (
            new MaskSegmentation(port_prefix, object_name, depth_stride, handle_mask_rpc)
        );
    }
    // Masks in non-real-time scenario taken from files from the dataset
    else if (segmentation_type == "masknrt")
    {
        segmentation = std::unique_ptr<MaskSegmentation>
        (
            new MaskSegmentation(port_prefix, object_data_path, object_name, depth_stride, segmentation_set)
        );
    }
    else
    {
        yError() << log_ID << "The requested segmentation is not available. Requested segmentation type is" << segmentation_type;
        std::exit(EXIT_FAILURE);
    }

    /**
     * Initialize point cloud prediction.
     */
    std::shared_ptr<PointCloudPrediction> point_cloud_prediction = std::make_shared<ObjectPointCloudPrediction>(object_point_cloud_path);

    /**
     * Initialize measurement model.
     */
    std::unique_ptr<AdditiveMeasurementModel> measurement_model;
    std::unique_ptr<ObjectMeasurements> obj_meas = std::unique_ptr<ObjectMeasurements>
    (
        new ObjectMeasurements(std::move(camera), std::move(segmentation), point_cloud_prediction, visual_covariance_diagonal, depth_fetch_mode)
    );
    obj_meas->enableOutlierRejection(pc_outlier_rejection);
    // if (enable_log)
    //     obj_meas->enable_log(log_path, "object-tracking");
    measurement_model = std::move(obj_meas);

    /* 2D Validation. */
    std::unique_ptr<Validator2D> validator;
    if (camera_name == "YCBVideoCameraNRT")
    {
        validator = std::unique_ptr<Validator2D>
        (
            new Validator2D(point_cloud_prediction, "object-tracking/validator2d", object_data_path)
        );
    }
    else
    {
        validator = std::unique_ptr<Validator2D>
        (
            new Validator2D(point_cloud_prediction, "object-tracking/validator2d", camera_name, camera_fallback_key)
        );
    }

    /**
     * Filter construction.
     */

    /**
     * StateModel
     */
    std::unique_ptr<LinearStateModel> kinematic_model = std::unique_ptr<DiscretizedKinematicModel>
    (
        new DiscretizedKinematicModel(kin_q_x(0), kin_q_x(1), kin_q_x(2), kin_q_eul(0), kin_q_eul(1), kin_q_eul(2), 1.0 / kin_rate, kin_est_period)
    );

    std::size_t dim_linear;
    std::size_t dim_circular;
    std::tie(dim_linear, dim_circular) = kinematic_model->getOutputSize();
    std::size_t state_size = dim_linear + dim_circular;

    /**
     * Initial condition.
     */

    VectorXd initial_covariance(12);
    initial_covariance.head<3>() = cov_x_0;
    initial_covariance.segment<3>(3) = cov_v_0;
    initial_covariance.segment<3>(6) = cov_eul_dot_0;
    initial_covariance.tail<3>() = cov_eul_0;
    std::unique_ptr<InitGroundTruth> initialization = std::unique_ptr<InitGroundTruth>
    (
        new InitGroundTruth(object_data_path, object_name, 0, initial_covariance.asDiagonal())
    );

    /**
     * Prediction step.
     */
    std::unique_ptr<GaussianPrediction> prediction;
    if (algorithm == "UKF")
        prediction = std::unique_ptr<KFPrediction>
        (
            new KFPrediction(std::move(kinematic_model))
        );
    else
        // Used for ICP
        prediction = std::unique_ptr<StaticPrediction>
        (
            new StaticPrediction()
        );

    /**
     * Correction step.
     */

    std::unique_ptr<GaussianCorrection> correction;
    if (algorithm == "UKF")
    {
        // A measurement is made of 3 * N scalars, N being the number of points
        std::size_t measurement_sub_size = 3;
        correction = std::unique_ptr<Correction>
        (
            new Correction(std::move(measurement_model), state_size, ut_alpha, ut_beta, ut_kappa, measurement_sub_size)
        );
    }
    else
        correction = std::unique_ptr<CorrectionICP>
        (
            new CorrectionICP(std::move(measurement_model), object_point_cloud_path)
        );

    /**
     * Filter.
     */
    std::cout << "Initializing filter..." << std::flush;

    std::unique_ptr<Filter> filter = std::unique_ptr<Filter>
    (
        new Filter(port_prefix,
                   std::move(initialization),
                   std::move(prediction),
                   std::move(correction),
                   std::move(validator))
    );
    if (enable_log)
        filter->enable_log(log_path, "object-tracking");

    /**
     * As explained in the paper, we initialize the filter from the first frame or
     * after the object was heavily occlued. In practice, the second approach is employed
     * only in case (i.e. video sequence 0059, object 010_potted_meat_can).
     */
    if ((object_name == "010_potted_meat_can") && (object_data_path.find("/0059") != std::string::npos))
        filter->skipFrames(200);

    std::cout << "done." << std::endl;

    std::cout << "Booting filter..." << std::flush;

    filter->boot();

    std::cout << "done." << std::endl;

    std::cout << "Running filter..." << std::endl;

    if (autostart)
        filter->run();

    if (!filter->wait())
        return EXIT_FAILURE;

    yInfo() << log_ID << "Application closed succesfully.";

    return EXIT_SUCCESS;
}


VectorXd loadVectorDouble(Bottle &rf, const std::string key, const std::size_t size)
{
    bool ok = true;

    if (rf.find(key).isNull())
        ok = false;

    Bottle* b = rf.find(key).asList();
    if (b == nullptr)
        ok = false;

    if (b->size() != size)
        ok = false;

    if (!ok)
    {
        yError() << "[Main]" << "Unable to load vector" << key;
        std::exit(EXIT_FAILURE);
    }

    VectorXd vector(size);
    for (std::size_t i = 0; i < b->size(); i++)
    {
        Value item_v = b->get(i);
        if (item_v.isNull())
            return VectorXd(0);

        if (!item_v.isDouble())
            return VectorXd(0);

        vector(i) = item_v.asDouble();
    }

    return vector;
}


std::vector<std::string> loadListString(Bottle &rf, const std::string key)
{
    bool ok = true;

    if (rf.find(key).isNull())
        ok = false;

    Bottle* b = rf.find(key).asList();
    if (b == nullptr)
        ok = false;

    if (!ok)
    {
        yError() << "[Main]" << "Unable to load list of strings with key" << key;
        std::exit(EXIT_FAILURE);
    }

    std::vector<std::string> list;
    for (std::size_t i = 0; i < b->size(); i++)
    {
        Value item_v = b->get(i);
        if (item_v.isNull())
            return std::vector<std::string>();

        if (!item_v.isString())
            return std::vector<std::string>();

        list.push_back(item_v.asString());
    }

    return list;
}


std::string eigenToString(const Ref<const VectorXd>& v)
{
    std::stringstream ss;
    ss << v.transpose();
    return ss.str();
}
