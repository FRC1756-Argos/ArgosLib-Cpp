/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/time.h>
#include <units/voltage.h>

#include "compile_time_member_check.h"
#include "ctre/Phoenix.h"

namespace argos_lib {
  namespace talonsrx_config {

    HAS_MEMBER(inverted)
    HAS_MEMBER(neutralMode)
    HAS_MEMBER(pid0_allowableError)
    HAS_MEMBER(pid0_iZone)
    HAS_MEMBER(pid0_kD)
    HAS_MEMBER(pid0_kF)
    HAS_MEMBER(pid0_kI)
    HAS_MEMBER(pid0_kP)
    HAS_MEMBER(pid0_selectedSensor)
    HAS_MEMBER(remoteFilter0_addr)
    HAS_MEMBER(remoteFilter0_type)
    HAS_MEMBER(sensorPhase)
    HAS_MEMBER(voltCompSat)

    /**
     * @brief Configures a CTRE TalonSRX with only the fields provided.  All other fields
     *        are given the factory default values.
     *
     * @tparam T Structure containing any combination of the following members:
     *           - inverted
     *           - neutralMode
     *           - pid0_allowableError
     *           - pid0_iZone
     *           - pid0_kD,
     *           - pid0_kF
     *           - pid0_kI
     *           - pid0_kP
     *           - pid0_selectedSensor
     *           - remoteFilter0_addr,
     *           - remoteFilter0_type
     *           - sensorPhase
     *           - voltCompSat
     * @param motorController TalonSRX object to configure
     * @param configTimeout Time to wait for response from TalonSRX
     * @return true Configuration succeeded
     * @return false Configuration failed
     */
    template <typename T>
    bool TalonSRXConfig(WPI_TalonSRX& motorController, units::millisecond_t configTimeout) {
      TalonSRXConfiguration config;
      auto timeout = configTimeout.to<int>();

      motorController.ConfigFactoryDefault(timeout);

      if constexpr (has_inverted<T>{}) {
        motorController.SetInverted(T::inverted);
      }
      if constexpr (has_sensorPhase<T>{}) {
        motorController.SetSensorPhase(T::sensorPhase);
      }
      if constexpr (has_neutralMode<T>{}) {
        motorController.SetNeutralMode(T::neutralMode);
      }
      if constexpr (has_voltCompSat<T>{}) {
        constexpr units::volt_t voltage = T::voltCompSat;
        config.voltageCompSaturation = voltage.to<double>();
        motorController.EnableVoltageCompensation(true);
      } else {
        motorController.EnableVoltageCompensation(false);
      }
      if constexpr (has_remoteFilter0_addr<T>{} && has_remoteFilter0_type<T>{}) {
        ctre::phoenix::motorcontrol::can::FilterConfiguration filterConfig;
        filterConfig.remoteSensorDeviceID = T::remoteFilter0_addr;
        filterConfig.remoteSensorSource = T::remoteFilter0_type;
        config.remoteFilter0 = filterConfig;
      }
      if constexpr (has_pid0_selectedSensor<T>{}) {
        config.primaryPID.selectedFeedbackSensor = T::pid0_selectedSensor;
      }
      if constexpr (has_pid0_kP<T>{}) {
        config.slot0.kP = T::pid0_kP;
      }
      if constexpr (has_pid0_kI<T>{}) {
        config.slot0.kI = T::pid0_kI;
      }
      if constexpr (has_pid0_kD<T>{}) {
        config.slot0.kD = T::pid0_kD;
      }
      if constexpr (has_pid0_kF<T>{}) {
        config.slot0.kF = T::pid0_kF;
      }
      if constexpr (has_pid0_iZone<T>{}) {
        config.slot0.integralZone = T::pid0_iZone;
      }
      if constexpr (has_pid0_allowableError<T>{}) {
        config.slot0.allowableClosedloopError = T::pid0_allowableError;
      }

      return 0 != motorController.ConfigAllSettings(config, timeout);
    }

  }  // namespace talonsrx_config
}  // namespace argos_lib
