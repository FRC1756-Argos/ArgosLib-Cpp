/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <units/time.h>

#include "compile_time_member_check.h"
#include "ctre/Phoenix.h"

namespace argos_lib {
  namespace cancoder_config {

    HAS_MEMBER(direction)
    HAS_MEMBER(initMode)
    HAS_MEMBER(magOffset)
    HAS_MEMBER(range)

    /**
     * @brief Configures a CTRE CanCoder with only the fields provided.  All other fields
     *        are given the factory default values.
     *
     * @tparam T Structure containing any combination of the following members:
     *           - direction
     *           - initMode
     *           - magOffset
     *           - range
     * @param encoder CANCoder object to configure
     * @param configTimeout Time to wait for response from CANCoder
     * @return true Configuration succeeded
     * @return false Configuration failed
     */
    template <typename T>
    bool CanCoderConfig(CANCoder& encoder, units::millisecond_t configTimeout) {
      ctre::phoenix::sensors::CANCoderConfiguration config;
      auto timeout = configTimeout.to<int>();

      if constexpr (has_direction<T>{}) {
        config.sensorDirection = T::direction;
      }
      if constexpr (has_initMode<T>{}) {
        config.initializationStrategy = T::initMode;
      }
      if constexpr (has_range<T>{}) {
        config.absoluteSensorRange = T::range;
      }
      if constexpr (has_magOffset<T>{}) {
        config.magnetOffsetDegrees = T::magOffset;
      }

      return 0 != encoder.ConfigAllSettings(config, timeout);
    }

  }  // namespace cancoder_config
}  // namespace argos_lib
