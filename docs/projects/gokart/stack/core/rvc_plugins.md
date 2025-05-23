---
title: Model Based Lon Control
hide_title: true
sidebar_position: 1
---

## Breakdown of RVC and Plugins
Race Vehicle Engine launches one node race_vehicle_controller. Inside of this node, during construction, it sets up the current state of the car through a SharedPtr. There are some misc, but the important part comes from the plugins.

The important part comes from this code section.

```cpp
  const auto plugin_names = declare_parameter("plugins", std::vector<std::string>{});
  for (const auto & plugin_name : plugin_names) {
    RvcPlugin::SharedPtr new_plugin = m_plugin_loader_.createSharedInstance(plugin_name);
    m_plugins_.push_back(new_plugin);
  }
  std::for_each(
    m_plugins_.begin(), m_plugins_.end(), [this](RvcPlugin::SharedPtr & p) {
      p->initialize(m_state_, m_config_, m_model_, this);
    });
  std::for_each(
    m_plugins_.begin(), m_plugins_.end(), [this](RvcPlugin::SharedPtr & p) {
      p->configure();
    });
```

The first loads in all the plugins specified in the parameter file where each is under plugins. You will notice that if you go to the plugins folder, at the very bottom of each source code, it exports the source code using the specified name referenced in the plugins section. For example, input_validation.cpp will be exported as race::InputValidation.

The second loops through the available plugins that were loaded in, and initializes their internal RVCState. This does NOT create a new copy but rather allows each plugin to have access to it. Therefore, any changes made to this by any other plugin or other nodes will cascade and be presented.

The third is a virtual method that is overrideable by the plugins. If you look at each plugin, you'll see that there's a configure(). This overrides the base class RVCPlugin and initializes all the necessary info for that specific plugin.

You will notice that RvcState has a function responsible for determining if the RVC node is ready. As such, the *kinematic_state, manual_cmd, and ttl_cmds are all linked to RVC.*

```cpp
  virtual void initialize(
    RvcState::SharedPtr state, RvcConfig::SharedPtr config,
    race::vehicle_model::VehicleModel::SharedPtr model,
    rclcpp::Node * node)
  {
    m_state_ = state;
    m_config_ = config;
    m_model_ = model;
    m_node_ = node;
  }

RvcState:
struct RvcState
{
  typedef std::shared_ptr<RvcState> SharedPtr;
  typedef std::unique_ptr<RvcState> UniquePtr;
  VehicleKinematicState::SharedPtr kin_state {};
  WheelSpeedReport::SharedPtr wheel_speed_report {};
  EngineReport::SharedPtr engine_report {};
  VehicleControlCommand::SharedPtr output_cmd {};
  VehicleManualControlCommand::SharedPtr input_manual_cmd {};
  TargetTrajectoryCommand::SharedPtr ttl_cmd {};
  RvcTelemetry::SharedPtr telemetry {};
  PathSharedPtr path {};
  DiagnosticArray::SharedPtr diagnostics {};
  Push2PassReport::SharedPtr push2pass_report {};

  bool all_input_received() const
  {
    return kin_state && input_manual_cmd && ttl_cmd;
  }
};

RvcConfig:
struct RvcConfig
{
  typedef std::shared_ptr<RvcConfig> SharedPtr;
  typedef std::unique_ptr<RvcConfig> UniquePtr;
  double control_output_interval_sec;
  double max_front_wheel_angle_rad;
  double wheelbase_m;   // Front-rear axle distance
  double track_m;   // Wheel-wheel distance on the same axle
  double vehicle_weight_kg;   // Vehicle weight
  double turn_left_bias_rad;  // left turn bias
  std::string ttl_directory;
};
}

def:
  RvcState::SharedPtr m_state_ {};
  RvcConfig::SharedPtr m_config_{};
  race::vehicle_model::VehicleModel::SharedPtr m_model_{};
  rclcpp::Node * m_node_ {};
```
### Note
Each plugin has their own respective parameters.

All RVCPlugin has a base function called *const_state* that just *const_casts* the current state of the car so it's immutable (not modifiable).

The plugins below uses pid_controller:
abs
lateral_comp
lon_control
model_based_lat
model_based_lon

### Model Based Lon
If you look inside of the source code for this, you will see a function called calc_lon_control(). This is essentially a high level calculation that computes the target speed specified in the parameter file and the current speed of the engine.

The only condition we need to worry about is cruise control as follow mode is for multiagent cars (this is good for next year). The cruise control is a trivial controls example where we follow the reference speed (target speed) and compute the error based on our current speed. As I said earlier, since model_based_lon *uses* PID Controller (inherits PID), it uses thsi error to compute the necessary PID

The code below is the core source code for controlling high level error for lon.
```cpp
if (lon_control_type_ == VehicleControlCommand::LON_CONTROL_THROTTLE) {
      if (const_state().engine_report) {
        // Update goal
        if (current_speed < stopping_speed_) {
          acc_pid_.reset_integral_error(0.0);
        }
        auto acc = 0.0;

        const auto stratergy_type = const_state().ttl_cmd->strategy_type.strategy_type;
        if (stratergy_type == race_msgs::msg::StrategyType::CRUISE_CONTROL) {
          acc = acc_pid_.update(
            target_speed - current_speed,
            const_config().control_output_interval_sec);
        } else if (stratergy_type == race_msgs::msg::StrategyType::FOLLOW_MODE) {
          const auto speed_diff = const_state().ttl_cmd->rival_car_speed - current_speed;
          const auto gap_diff = const_state().ttl_cmd->target_gap -
            const_state().ttl_cmd->rival_car_gap;
          const auto gap_term = std::clamp(
            follow_mode_max_gap_diff_ratio_ * gap_diff,
            -follow_mode_max_speed_diff_,
            follow_mode_max_speed_diff_);
          const auto target_speed_diff = std::min(
            speed_diff - gap_term,
            target_speed - current_speed);
          acc = acc_pid_.update(target_speed_diff, const_config().control_output_interval_sec);
          state().telemetry->target_speed = target_speed_diff;
        } else {
          RCLCPP_WARN_THROTTLE(
            node().get_logger(),
            *node().get_clock(), 500,
            "Unknown stratergy type in TargetTrajectoryCommand of %u. Stopping.",
            stratergy_type);
          acc = acc_pid_.update(
            0.0 - current_speed,
            const_config().control_output_interval_sec);
        }

        // limit the stopping dcc in norminal and safe stop
        if (stop_type == race_msgs::msg::StopType::STOP_TYPE_NOMINAL ||
          stop_type == race_msgs::msg::StopType::STOP_TYPE_SAFE)
        {
          acc = std::clamp(acc, norminal_max_dcc_, acc);
        }

        // calculate lon control
        auto output = model().calc_lon_control(
          race::vehicle_model::VehicleModelState{
            current_speed,
            slip_angle_,
            const_state().path->back().curvature,
            const_state().kin_state->accel.accel.linear.x,
            const_state().kin_state->accel.accel.linear.y,
            const_state().kin_state->car_yaw_rate,
            const_state().kin_state->front_wheel_angle_rad,
            const_state().path->back().bank_angle,
            const_state().engine_report->engine_rpm,
            static_cast<size_t>(const_state().engine_report->current_gear)
          }, acc);
        output.throttle_level *= throttle_scale_;
        output.throttle_level = std::clamp(output.throttle_level, 0.0, throttle_max_);
        slip_angle_ = output.slip_angle_rad;

        state().telemetry->throttle_cmd = output.throttle_level;
        state().telemetry->brake_cmd = output.brake_psi;
        state().telemetry->target_lon_acc = acc;

        if (output.brake_psi > 0.0 && output.brake_psi > brake_deadband_psi_) {
          output.throttle_level = 0.0;
          // filter output
          output.brake_psi = brake_filter_.update(output.brake_psi);
        } else {
          // filter output
          output.throttle_level = throttle_filter_.update(output.throttle_level);
          output.brake_psi = 0.0;
        }
        // put parking brake
        if (current_speed < stopping_speed_ && target_speed < stopping_speed_) {
          RCLCPP_INFO_THROTTLE(
            node().get_logger(),
            *(node().get_clock()), 1000, "Vehicle stopped and target speed zero. Holding brake.");
          output.throttle_level = 0.0;
          output.brake_psi = parking_brake_psi_;
        }

        output_cmd.accelerator_cmd = output.throttle_level;
        output_cmd.brake_cmd = output.brake_psi;
        output_cmd.push2pass_cmd = const_state().ttl_cmd->push2pass_cmd |
          const_state().input_manual_cmd->vehicle_control_command.push2pass_cmd;

        if (test_mode_) {
          if (output_cmd.brake_cmd > 0.0) {
            output_cmd.accelerator_cmd = 0.0;
            if (const_max_brake_) {
              output_cmd.brake_cmd = max_brake_;
            } else {
              output_cmd.brake_cmd = std::min(output_cmd.brake_cmd, max_brake_);
            }
            state().telemetry->throttle_cmd = output_cmd.accelerator_cmd;
            state().telemetry->brake_cmd = output_cmd.brake_cmd;
          }
        }
      }
    } else {
      output_cmd.speed_cmd = target_speed;
      output_cmd.lon_control_type = VehicleControlCommand::LON_CONTROL_SPEED;
      output_cmd.push2pass_cmd = const_state().ttl_cmd->push2pass_cmd |
        const_state().input_manual_cmd->vehicle_control_command.push2pass_cmd;
    }
  }
```

This below is the PIDController for updating the command to fix error.
```
double PidController::update(double new_error, double actual_dt)
{
  if (std::isnan(new_error)) {
    RCLCPP_WARN(rclcpp::get_logger(name_), "WARNING: error -> NAN");
    return NAN;
  }

  last_error_ = error_;
  error_ = new_error;

  // Calculate & saturate integral error.

  integral_error_ += error_ * actual_dt;
  integral_error_ = std::clamp(integral_error_, coefficients_.min_i, coefficients_.max_i);

  // Calculate dt error.

  const auto dt_error = (error_ - last_error_) / actual_dt;

  // Calculate control.

  const auto p = error_ * coefficients_.k_p;
  const auto i = integral_error_ * coefficients_.k_i;
  const auto d = dt_error * coefficients_.k_d;

  const auto cmd = p + i + d;

  // Clamp control.

  if (cmd <= coefficients_.min_cmd) {
    // RCLCPP_WARN(
    //   rclcpp::get_logger(
    //     name_), "WARNING: clamped %f to min %f", cmd, coefficients_.min_cmd);
    return coefficients_.min_cmd;
  }

  if (cmd >= coefficients_.max_cmd) {
    // RCLCPP_WARN(
    //   rclcpp::get_logger(
    //     name_), "WARNING: clamped %f to max %f", cmd, coefficients_.max_cmd);
    return coefficients_.max_cmd;
  }

  return cmd;
}
```

Below is the low level telemetry that connects to high level model based lon control.

```cpp
LonControlOutput VehicleModel::calc_lon_control(
  const VehicleModelState & state,
  const double & target_acc)
{
  try {
    const auto limited_target_acc =
      std::min(abs(target_acc), get_available_lon_acc(state) / 2) * (target_acc > 0 ? 1.0 : -1.0);
    const auto net_force = calc_net_force(limited_target_acc);
    const auto resistance = calc_resistance(state.speed_mps, state.bank_angle);
    // control_f + resistance_f = net_f
    const auto control_force = net_force - resistance;

    const auto target_wheel_torque = control_force * config_->rear_tyre_config->radius;
    const auto target_engine_torque = target_wheel_torque /
      (config_->powertrain_config->gear_ratio[state.gear_num - 1] *
      config_->powertrain_config->final_drive_ratio);
    const auto min_engine_torque = bilinear_interpolate(
      config_->powertrain_config->torque_v_rpm_throttle, state.engine_rpm, 0.0, false);

    const auto min_control_force = min_engine_torque *
      (config_->powertrain_config->gear_ratio[state.gear_num - 1] *
      config_->powertrain_config->final_drive_ratio) / config_->rear_tyre_config->radius;
    // if (control_force >= 0) {
    if (target_engine_torque >= min_engine_torque) {
      // If control force is possitive, throttle up.
      return LonControlOutput{calc_throttle(state.engine_rpm, state.gear_num, control_force), 0.0,
        0.0};
    } else {
      // If control force is negative, brake.
      return LonControlOutput{0.0, calc_brake(control_force - min_control_force), 0.0};
    }
  } catch (const std::exception & e) {
    printf("%s", e.what());
  }
  return LonControlOutput{0.0, 0.0, 0.0};
}
```
Low level telemetry

### Lateral Compensation
This gets called after all the plugins responsible for computing control command. This plugin has a function called modify_control_command which is ran after all the important stuff first.

```cpp
bool modify_control_command(VehicleControlCommand & input_cmd) override
  {
    if (!enabled_) {
      return true;
    }

    if (!const_state().telemetry->mpc_enabled) {
      return true;
    }

    if (true) {
      double grad_ov_ud = 0.0;
      for (int i = 0; i < curr_ws.size(); i++) {
        double curr_w = curr_ws[i];
        double curr_steer = curr_steers[i];
        double curr_vx = curr_vxs[i];
        // auto speed = const_state().kin_state->speed_mps;
        if (curr_vx < 0.0 || isnan(curr_vx)) {  // check for stationary / nan value
          curr_vx = 0.0;
        }
        const auto adj_ratio = utils::interpolate(
          ov_ud_lookup_speed, ov_ud_lookup_ratio, curr_vx,
          true);

        double curr_w_ = (1 - adj_ratio + learnable_ov_ud) * curr_w;
        double expected_w = tan(curr_steer) * curr_vx / const_config().wheelbase_m;
        grad_ov_ud += 2 * (curr_w_ - expected_w) * curr_w;
      }
      learnable_ov_ud -= grad_ov_ud * learning_rate / std::max(1, static_cast<int>(curr_ws.size()));
      learnable_ov_ud = std::clamp(learnable_ov_ud, learnable_ov_ud_min, learnable_ov_ud_max);
      std_msgs::msg::Float64 learnable_msg;
      learnable_msg.data = learnable_ov_ud;
      learnable_ov_ud_publisher_->publish(learnable_msg);
      double w = const_state().kin_state->car_yaw_rate;
      double vx = const_state().kin_state->speed_mps;
      double steer = const_state().kin_state->front_wheel_angle_rad - steer_bias;
      if (abs(w) > min_w && vx > min_vx) {
        curr_ws.push_back(w);
        curr_steers.push_back(steer);
        curr_vxs.push_back(vx);
      }
      if (curr_ws.size() > buffer_size) {
        curr_ws.erase(curr_ws.begin());
        curr_steers.erase(curr_steers.begin());
        curr_vxs.erase(curr_vxs.begin());
      }
    }

    input_cmd.steering_cmd += compensate_lateral_error(const_state().telemetry->lateral_error);
    input_cmd.steering_cmd = compensate_ov_ud_steer(input_cmd.steering_cmd);

    return true;
  }
```

This section of code is responsible for fixing the lateral error through the steering cmd

