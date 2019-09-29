/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

/**
 * ObjectTrackingIDL
 *
 */

service ObjectTrackingIDL
{
    /**
     * Initialize and run the filter.
     *
     * @return true/false on success/failure.
     */
    bool run_filter();

    /**
     * Reset the filter.
     *
     * @return true/false on success/failure.
     */
    bool reset_filter();

    /**
     * Stop and reset the filter.
     *
     * @return true/false on success/failure.
     */
    bool stop_filter();

    /**
     * Pause the filter.
     *
     */
    void pause_filter();

    /**
     * Resume the filter.
     *
     */
    void resume_filter();

    /**
     * Enable/Disable skipping the filtering step specified in what_step.
     * what_step can be one of the following:
     *
     *  1) prediction: skips the whole prediction step
     *
     *  2) state: skips the prediction step related to the state transition
     *
     *  3) exogenous: skips the prediction step related exogenous inputs
     *
     *  4) correction: skips the whole correction step
     *
     * @param what_step the step to skipping
     * @param status enable/disable skipping
     *
     * @return true/false on success/failure.
     */
    bool skip_step(1:string what_step, 2:bool status);

    /**
     * Disabla logging and saves opened log files if any.
     */
    bool disable_logs();

    /**
     * Quit the filter in graceful way.
     */
    bool quit();
}
