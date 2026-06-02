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

#include "EKF2SourceArbiter.hpp"


// Initial schedule interval; rescheduled to ASA_RATE on the first Run() tick.
static constexpr uint32_t SCHEDULE_INTERVAL_US{100_ms};

static const char *state_name(EKF2SourceArbiter::State s) {
  static constexpr const char *names[] = {"IDLE", "EV", "->GPS", "GPS",
                                          "->EV"};
  const uint8_t idx = static_cast<uint8_t>(s);
  return (idx < 5) ? names[idx] : "?";
}


// #region Lifecycle

EKF2SourceArbiter::EKF2SourceArbiter()
    : ModuleParams(nullptr),
      ScheduledWorkItem(MODULE_NAME,
                        // nav_and_controllers: highest priority after sensors
                        px4::wq_configurations::nav_and_controllers) {
  // get params
  _hdl_ev_ctrl = param_find("EKF2_EV_CTRL");
  _hdl_gps_ctrl = param_find("EKF2_GPS_CTRL");
  _hdl_ev_noise_md = param_find("EKF2_EV_NOISE_MD");
  _hdl_evp_noise = param_find("EKF2_EVP_NOISE");

  if (_hdl_ev_ctrl == PARAM_INVALID || _hdl_gps_ctrl == PARAM_INVALID) {
    PX4_ERR("EKF2_EV_CTRL or EKF2_GPS_CTRL not found - is EKF2 built?");
  }

  if (_hdl_evp_noise != PARAM_INVALID) {
    param_get(_hdl_evp_noise, &_evp_noise_baseline);
  }

  _perf_run = perf_alloc(PC_ELAPSED, MODULE_NAME ": run");
}

EKF2SourceArbiter::~EKF2SourceArbiter() {
  // ScheduleClear(); // unsure is rqeuired here
  perf_free(_perf_run);
}

int EKF2SourceArbiter::task_spawn(int argc, char *argv[]) {
  EKF2SourceArbiter *instance = new EKF2SourceArbiter();

  if (instance) {
    _object.store(instance);
    _task_id = task_id_is_work_queue;

    // Kick off the work item; Run() reschedules to ASA_RATE on its first tick.
    instance->ScheduleOnInterval(SCHEDULE_INTERVAL_US);

    return PX4_OK;

  } else {
    PX4_ERR("alloc failed");
  }

  delete instance;
  _object.store(nullptr);
  _task_id = -1;

  return PX4_ERROR;
}

// #endregion

// #region Paramater Application

void EKF2SourceArbiter::applyParams() {

  if (_hdl_ev_ctrl == PARAM_INVALID || _hdl_gps_ctrl == PARAM_INVALID) {
    return;
  }

  // get existing values
  int32_t ev_ctrl = 0;
  int32_t gps_ctrl = 0;
  param_get(_hdl_ev_ctrl, &ev_ctrl);
  param_get(_hdl_gps_ctrl, &gps_ctrl);

  // get target values from our own paramaters
  const int32_t ev_bits = (int32_t)_param_ev_bits.get();
  const int32_t gps_bits = (int32_t)_param_gps_bits.get();

  // handle param changes
  int32_t ev_new = ev_ctrl;
  int32_t gps_new = gps_ctrl;

  // TODO: SLAM_* -> EV_*
  switch (_state) {
  case State::EV_ONLY:
    ev_new = ev_ctrl | ev_bits;     // EV  bits ON
    gps_new = gps_ctrl & ~gps_bits; // GPS bits OFF
    break;

  case State::BLEND_TO_GPS:
  case State::BLEND_TO_EV:
    ev_new = ev_ctrl | ev_bits;    // EV  bits ON
    gps_new = gps_ctrl | gps_bits; // GPS bits ON
    break;

  case State::GPS_FALLBACK:
    ev_new = ev_ctrl & ~ev_bits;   // EV  bits OFF
    gps_new = gps_ctrl | gps_bits; // GPS bits ON
    break;

  case State::IDLE:
    break;
  }

  if (ev_new != ev_ctrl) {
    param_set(_hdl_ev_ctrl, &ev_new);
  }
  if (gps_new != gps_ctrl) {
    param_set(_hdl_gps_ctrl, &gps_new);
  }
}

// #endregion

// When blending to GPS, we ramp EKF2_EVP_NOISE from its baseline to
// EV_NOISE_RAMP_MAX so EKF2 smoothly downweights visual odometry before EV is
// disabled.
//
// In EV_ONLY the baseline is restored and EKF2_EV_NOISE_MD is set back to 0
// (use message covariance) so normal EV operation is unaffected.
//
// EKF2_EV_NOISE_MD=1 forces EKF2 to use EKF2_EVP_NOISE instead of the
// covariance supplied in the vehicle_visual_odometry message.
void EKF2SourceArbiter::applyNoiseRamp(hrt_abstime now, hrt_abstime blend_t) {
  // check that we have handles to change params with
  if (_hdl_ev_noise_md == PARAM_INVALID || _hdl_evp_noise == PARAM_INVALID) {
    return;
  }

  float target_noise;
  int32_t target_md;

  switch (_state) {
  case State::BLEND_TO_GPS: {
    const float noise_max = _param_ev_noise_max.get();
    const float t = (blend_t > 0)
                        ? fmaxf(0.f, fminf(1.f, (float)(now - _blend_start) /
                                                    (float)blend_t))
                        : 1.f;
    target_noise = _evp_noise_baseline + t * (noise_max - _evp_noise_baseline);
    target_md = 1;
    break;
  }

  case State::BLEND_TO_EV: {
    const float noise_max = _param_ev_noise_max.get();
    const float t = (blend_t > 0)
                        ? fmaxf(0.f, fminf(1.f, (float)(now - _blend_start) /
                                                    (float)blend_t))
                        : 1.f;
    target_noise = noise_max - t * (noise_max - _evp_noise_baseline);
    target_md = 1;
    break;
  }

  case State::EV_ONLY:
    target_noise = _evp_noise_baseline;
    target_md = 0;
    break;

  default:
    return; // GPS_FALLBACK: EV is off, nothing to ramp
  }

  int32_t cur_md = 0;
  float cur_noise = 0.f;
  param_get(_hdl_ev_noise_md, &cur_md);
  param_get(_hdl_evp_noise, &cur_noise);

  if (cur_md != target_md) {
    param_set(_hdl_ev_noise_md, &target_md);
  }
  if (fabsf(cur_noise - target_noise) > 0.01f) {
    param_set(_hdl_evp_noise, &target_noise);
  }
}

// The arbiter module starts in IDLE state until it is enabled
// by the companion computer using VEHICLE_CMD_EKF2_ARBITER_SET
// (command 43001 with param1 >= 1) before the module controls EKF2 params.
// The same command with param==0 returns the module to an IDLE state.
//
// param2 0.0f - no action
// param2 1.0f - force to GPS now
// param2 2.0f - force to EV now
// param2 does not prevent FC from switching sources as usual
//
// Commands are ack'd via vehicle_command_ack.
void EKF2SourceArbiter::handleVehicleCommands() {
  vehicle_command_s cmd{};

  while (_vehicle_cmd_sub.update(&cmd)) {
    if (cmd.command != VEHICLE_CMD_EKF2_ARBITER_SET) {
      continue;
    }

    if (cmd.param1 >= 1.f) {
      if (_state == State::IDLE) {
        _state = State::EV_ONLY;
        _ev_lost_since = 0;
        mavlink_log_info(&_mavlink_log_pub, "[arbiter] enabled by companion");
      }
      // Allow external computer to trigger a state change
      if (_state != State::IDLE && PX4_ISFINITE(cmd.param2)) {
        if (cmd.param2 >= 1.5f) {// force to EV
          _external_commanded_state = State::BLEND_TO_EV;
	  mavlink_log_info(&_mavlink_log_pub, "[arbiter] ack'd EV switch request");
        } else if (cmd.param2 >= 0.5f) {
          _external_commanded_state = State::BLEND_TO_GPS;
	  mavlink_log_info(&_mavlink_log_pub, "[arbiter] ack'd GPS switch request");
        }
      }
    } else {
      if (_state != State::IDLE) {
        _state = State::IDLE;
	_external_commanded_state = State::IDLE;
        mavlink_log_info(&_mavlink_log_pub, "[arbiter] disabled by companion");
      }
    }

    vehicle_command_ack_s ack{};
    ack.timestamp = hrt_absolute_time();
    ack.command = cmd.command;
    ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
    ack.target_system = cmd.source_system;
    ack.target_component = cmd.source_component;
    _cmd_ack_pub.publish(ack);
  }
}

// Status publication
void EKF2SourceArbiter::publishStatus(hrt_abstime now, hrt_abstime blend_t) {
  ekf2_source_arbiter_status_s status{};
  status.timestamp = now;
  status.state = static_cast<uint8_t>(_state);
  status.ev_pipeline_alive = isEVPipelineAlive();
  status.ev_being_fused = isEVBeingFused();
  status.primary_ekf_instance = _primary_instance;

  float cur_noise = _evp_noise_baseline;

  if (_hdl_evp_noise != PARAM_INVALID) {
    param_get(_hdl_evp_noise, &cur_noise);
  }

  status.ev_noise_current = cur_noise;

  if ((_state == State::BLEND_TO_GPS || _state == State::BLEND_TO_EV) &&
      blend_t > 0) {
    status.blend_progress =
        fmaxf(0.f, fminf(1.f, (float)(now - _blend_start) / (float)blend_t));
  } else {
    status.blend_progress = 0.f;
  }

  _status_pub.publish(status);
}

// #region EV health checking

bool EKF2SourceArbiter::isEVPipelineAlive() {
  const hrt_abstime now = hrt_absolute_time();
  const hrt_abstime stale_age = (hrt_abstime)(_param_ev_stale_t.get() * 1e6f);
  vehicle_odometry_s vo{};
  _visual_odom_sub.copy(&vo);
  return vo.timestamp != 0 && (now - vo.timestamp) <= stale_age;
}

bool EKF2SourceArbiter::isEVBeingFused() {
  if (!isEVPipelineAlive()) {
    return false;
  }

  estimator_aid_source2d_s ev_pos{};

  if (!_ev_pos_subs[_primary_instance].copy(&ev_pos)) {
    return false;
  }

  if (ev_pos.timestamp == 0) {
    return false;
  }

  const hrt_abstime now = hrt_absolute_time();
  const hrt_abstime stale_age = (hrt_abstime)(_param_ev_stale_t.get() * 1e6f);
  return ev_pos.fused && (now - ev_pos.time_last_fuse) < stale_age;
}

// #endregion

// #region main loop

void EKF2SourceArbiter::Run() {
  // PX4 glue logic

  if (should_exit()) {
    ScheduleClear();
    exit_and_cleanup();
    return;
  }

  perf_begin(_perf_run);

  if (_parameter_update_sub.updated()) {
    parameter_update_s pup;
    _parameter_update_sub.copy(&pup);
    updateParams();
  }

  // Our logic begins

  // Reschedule if RATE param was changed.
  const float rate_hz = _param_rate.get();

  if (fabsf(rate_hz - _last_rate_hz) > 0.01f) {
    _last_rate_hz = rate_hz;
    ScheduleOnInterval((rate_hz > 0.f) ? (uint32_t)(1e6f / rate_hz)
                                       : SCHEDULE_INTERVAL_US);
  }

  const hrt_abstime now = hrt_absolute_time();
  const hrt_abstime ev_tout = (hrt_abstime)(_param_ev_tout.get() * 1e6f);
  const hrt_abstime blend_t = (hrt_abstime)(_param_blend_t.get() * 1e6f);
  const hrt_abstime recov_t = (hrt_abstime)(_param_gps_recov_t.get() * 1e6f);

  // Keep primary EKF instance up-to-date regardless of arbiter state.
  estimator_selector_status_s sel{};

  if (_selector_status_sub.update(&sel)) {
    _primary_instance = sel.primary_instance;
  }

  handleVehicleCommands();

  if (_state != State::IDLE) {
    // if EV enabled, use isEVBeingFused (pipeline+EKF2 quality)
    // or isEVPipelineAlive() if not enabled.

    // Note that isEVBeingFused() will return false if
    // !isEVPipelineAlive() (we don't want disabled sources to be fused)
    const bool ev_alive = isEVPipelineAlive();
    const bool ev_fusing = isEVBeingFused();
    const State prev = _state;

    switch (_state) {
    case State::EV_ONLY:
      // check if we have been instructed by ext. computer to switch to GPS
      if (_external_commanded_state == State::BLEND_TO_GPS) {
	    // If we have a noise param set, fetch it
        if (_hdl_evp_noise != PARAM_INVALID) {
          param_get(_hdl_evp_noise, &_evp_noise_baseline);
        }

        // Start blend to GPS fallback
        _state = State::BLEND_TO_GPS;
        _blend_start = now;
	_external_commanded_state = State::IDLE;
	break;
      }

      // if we are currently fusing, mark this instant as when we lost EV.
      if (ev_fusing) {
        _ev_lost_since = 0;

      } else {
        if (_ev_lost_since == 0) {
          _ev_lost_since = now;
        }

        if ((now - _ev_lost_since) >= ev_tout) {
          // We have lost EV for more than the timeout

          // If we have a noise param set, fetch it
          if (_hdl_evp_noise != PARAM_INVALID) {
            param_get(_hdl_evp_noise, &_evp_noise_baseline);
          }

          // Start blend to GPS fallback
          _state = State::BLEND_TO_GPS;
          _blend_start = now;
        }
      }
      break;

    case State::GPS_FALLBACK:
      // check if we have been instructed by ext. computer to switch to EV
      if (_external_commanded_state == State::BLEND_TO_EV) {
          _ev_good_since = now;
	  _state       = State::BLEND_TO_EV;
          _blend_start = now;
	  _external_commanded_state = State::IDLE;
	  break;
      }

      if (ev_alive) {
        if (_ev_good_since == 0) {
          _ev_good_since = now;
        }

        if ((now - _ev_good_since) >= recov_t) {
          _state       = State::BLEND_TO_EV;
          _blend_start = now;
        }

      } else {
        _ev_good_since = 0;
      }
      break;

    case State::BLEND_TO_GPS:
      // Both EV and GPS are fused; EKF2 downgrades EV as it degrades

      if (ev_fusing) {
        // VO restored (healthy) during blend, restore state
        _state = State::EV_ONLY;
        _ev_lost_since = 0;

      } else if ((now - _blend_start) >= blend_t) {
        // Blend complete; VO did not recover, fallback to GPS
        _state = State::GPS_FALLBACK;
        _ev_good_since = 0;
      }
      break;

    case State::BLEND_TO_EV:
      // Both sources active, EKF2 should shift back to EV over time.
      // EV may just have been re-enabled so keep checking until it is healthy.
      if (!ev_alive) {
        // EV dropped during recovery blend, back to GPS
        _state = State::GPS_FALLBACK;
        _ev_good_since = 0;

      } else if ((now - _blend_start) >= blend_t) {
        // Blend complete; VO has recovered.
        _state = State::EV_ONLY;
        _ev_lost_since = 0;
      }

      break;

    default:
      // no action on default
      break;
    }

    // handle state changes

    // mavlink logging (shows in QGC)
    if (_state != prev) {
      if (_state == State::BLEND_TO_GPS) {
        mavlink_log_warning(&_mavlink_log_pub,
                            "[arbiter] EV lost, blending GPS");

      } else if (_state == State::GPS_FALLBACK) {
        mavlink_log_warning(&_mavlink_log_pub,
                            "[arbiter] EV lost - GPS fallback");

      } else if (_state == State::BLEND_TO_EV) {
        mavlink_log_info(&_mavlink_log_pub, "[arbiter] EV avail, blending in");

      } else if (_state == State::EV_ONLY) {
        mavlink_log_info(&_mavlink_log_pub, "[arbiter] EV stable, GPS off");
      }
    }

    // Enforce the correct EKF2 parameter configuration for the current state
    // every cycle; corrects any external override within one tick.
    applyParams();
    applyNoiseRamp(now, blend_t);
  }

  publishStatus(now, blend_t);
  perf_end(_perf_run);
}

// #endregion

// #region CLI commands and module desc

// ── boilerplate
// ───────────────────────────────────────────────────────────────

int EKF2SourceArbiter::print_status() {
  PX4_INFO("state        : %s", state_name(_state));
  PX4_INFO("primary EKF  : %u", (unsigned)_primary_instance);

  int32_t ev_ctrl = 0;
  int32_t gps_ctrl = 0;

  if (_hdl_ev_ctrl != PARAM_INVALID) {
    param_get(_hdl_ev_ctrl, &ev_ctrl);
  }
  if (_hdl_gps_ctrl != PARAM_INVALID) {
    param_get(_hdl_gps_ctrl, &gps_ctrl);
  }

  PX4_INFO("EKF2_EV_CTRL : 0x%02" PRIx32, ev_ctrl);
  PX4_INFO("EKF2_GPS_CTRL: 0x%02" PRIx32, gps_ctrl);

  perf_print_counter(_perf_run);
  return 0;
}

int EKF2SourceArbiter::custom_command(int argc, char *argv[]) {
  return print_usage("unknown command");
}

int EKF2SourceArbiter::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
EV (primary) / GPS (fallback) position aid source arbiter for EKF2.
)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("ekf2_source_arbiter", "estimator");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}

// #endregion

extern "C" __EXPORT int ekf2_source_arbiter_main(int argc, char *argv[]) {
  return EKF2SourceArbiter::main(argc, argv);
}
