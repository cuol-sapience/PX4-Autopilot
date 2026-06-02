/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/estimator_aid_source2d.h>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/ekf2_source_arbiter_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

using namespace time_literals;

// extern "C" __EXPORT int template_module_main(int argc, char *argv[]);

class EKF2SourceArbiter : public ModuleBase<EKF2SourceArbiter>,
                          public ModuleParams,
                          public px4::ScheduledWorkItem {
public:
  EKF2SourceArbiter();

  virtual ~EKF2SourceArbiter() override;

  /** @see ModuleBase */
  static int task_spawn(int argc, char *argv[]);

  /** @see ModuleBase */
  static EKF2SourceArbiter *instantiate(int argc, char *argv[]);

  /** @see ModuleBase */
  static int custom_command(int argc, char *argv[]);

  /** @see ModuleBase */
  static int print_usage(const char *reason = nullptr);

  /** @see ModuleBase::print_status() */
  int print_status() override;

  // Blending state
  enum class State : uint8_t {
    IDLE = 0,         // running, waiting for enable command from companion
    EV_ONLY = 1,      // EV pos on,  GPS pos/vel off
    BLEND_TO_GPS = 2, // EV pos on,  GPS pos/vel on  (EV degraded)
    GPS_FALLBACK = 3, // EV pos off, GPS pos/vel on
    BLEND_TO_EV = 4,  // EV pos on,  GPS pos/vel on  (EV recovering)
  };

private:
  void Run() override;

  // MAVLink vehicle_command ID accepted by this module.
  // param1 >= 1 = enable; param1 == 0 = disable.
  static constexpr uint16_t VEHICLE_CMD_EKF2_ARBITER_SET = 43001;

  // True when the SLAM pipeline is alive (odometry is fresh).
  bool isEVPipelineAlive();

  // True when the pipeline is alive AND EKF2 is actively fusing the data.
  // Only valid in states where EV_POS_BITS are enabled; always returns false
  // when EV position is disabled (GPS_FALLBACK / early BLEND_TO_SLAM).
  bool isEVBeingFused();

  // Read-modify-write EKF2_EV_CTRL and EKF2_GPS_CTRL to match the
  // current state.  No-op when values are already correct.
  void applyParams();

  // Linearly ramp EKF2_EVP_NOISE over the blend window so EKF2 shifts
  // position weight between EV and GPS without a hard innovation spike.
  void applyNoiseRamp(hrt_abstime now, hrt_abstime blend_t);

  // Process any pending vehicle_command messages and send acks.
  void handleVehicleCommands();

// Publish the current arbiter state to the ekf2_source_arbiter_status topic.
  void publishStatus(hrt_abstime now, hrt_abstime blend_t);


  /**
   * Check for parameter changes and update them if needed.
   * @param parameter_update_sub uorb subscription to parameter_update
   * @param force for a parameter update
   */
  // void parameters_update(bool force = false);

  perf_counter_t _perf_run{};

  DEFINE_PARAMETERS(
      (ParamFloat<px4::params::ESA_EV_TOUT>)_param_ev_tout,
      (ParamFloat<px4::params::ESA_BLEND_T>)_param_blend_t,
      (ParamFloat<px4::params::ESA_GPS_RECOV_T>)_param_gps_recov_t,
      (ParamFloat<px4::params::ESA_EV_STALE_T>)_param_ev_stale_t,
      (ParamFloat<px4::params::ESA_EV_NOISE_MAX>)_param_ev_noise_max,
      //(ParamFloat<px4::params::ASA_MLOG_PERIOD>)_param_mlog_period,
      (ParamFloat<px4::params::ESA_RATE>)_param_rate,
      (ParamInt<px4::params::ESA_EV_BITS>)_param_ev_bits,
      (ParamInt<px4::params::ESA_GPS_BITS>)_param_gps_bits);

  orb_advert_t _mavlink_log_pub{nullptr};

  State _state{State::IDLE};
  State _external_commanded_state{State::IDLE};

  // Timer tracking continuous EV failure in EV_ONLY (0 = not started).
  hrt_abstime _ev_lost_since{0};

  // Timer tracking how long we have been in a blend state.
  hrt_abstime _blend_start{0};

  // Timer tracking continuous EV health in GPS_FALLBACK (0 = not started).
  hrt_abstime _ev_good_since{0};

  // Cached parameter handles for EKF2 module params.
  param_t _hdl_ev_ctrl{PARAM_INVALID};
  param_t _hdl_gps_ctrl{PARAM_INVALID};
  param_t _hdl_ev_noise_md{PARAM_INVALID};
  param_t _hdl_evp_noise{PARAM_INVALID};

  // EKF2_EVP_NOISE value read at blend entry; restored when EV is stable.
  float _evp_noise_baseline{0.1f};

  // EKF primary instance index (from estimator_selector_status).
  uint8_t _primary_instance{0};

   float _last_rate_hz{0.f};

  // #region Subscriptions

  uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update),
                                                   1_s};

  uORB::Subscription _selector_status_sub{ORB_ID(estimator_selector_status)};

  // Direct input from the EV pipeline via micro-XRCE-DDS.
  // Watching the raw odometry topic lets us detect a dead agent or crashed
  // EV/SLAM node ~300 ms faster than waiting for EKF2's internal 400 ms
  // timeout.
  uORB::Subscription _visual_odom_sub{ORB_ID(vehicle_visual_odometry)};

  // Aid source feedback - one slot per EKF instance (up to 4).
  uORB::SubscriptionMultiArray<estimator_aid_source2d_s, 4> _ev_pos_subs{
      ORB_ID::estimator_aid_src_ev_pos};

  // Vehicle command to allow starting from companion computer
  uORB::Subscription _vehicle_cmd_sub{ORB_ID(vehicle_command)};

  // #endregion

  // #region Publications

  uORB::Publication<ekf2_source_arbiter_status_s> _status_pub{
      ORB_ID(ekf2_source_arbiter_status)};
  uORB::Publication<vehicle_command_ack_s> _cmd_ack_pub{
      ORB_ID(vehicle_command_ack)};

  // #endregion
};
