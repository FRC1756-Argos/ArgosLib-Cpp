/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/controller/xbox_controller.h"

using namespace argos_lib;

XboxController::XboxController(int port) : frc::GenericHID(port), m_vibrationModel(VibrationOff()) {
  m_buttonDebounceSettings.fill({0_ms, 0_ms});
  m_buttonDebounceStatus.fill(false);
  m_rawButtonStatus.fill(false);
  m_buttonDebounceTransitionTime.fill(std::chrono::steady_clock::now());
}

void XboxController::SetButtonDebounce(Button targetButton, DebounceSettings newSettings) {
  m_buttonDebounceSettings.at(static_cast<int>(targetButton)) = newSettings;
}

void XboxController::SwapSettings(XboxController& other) {
  std::swap(m_buttonDebounceSettings, other.m_buttonDebounceSettings);
  std::fill(m_rawButtonStatus.begin(), m_rawButtonStatus.end(), false);
  std::fill(other.m_rawButtonStatus.begin(), other.m_rawButtonStatus.end(), false);
  std::fill(m_buttonDebounceStatus.begin(), m_buttonDebounceStatus.end(), false);
  std::fill(other.m_buttonDebounceStatus.begin(), other.m_buttonDebounceStatus.end(), false);
  const auto now{std::chrono::steady_clock::now()};
  std::fill(m_buttonDebounceTransitionTime.begin(), m_buttonDebounceTransitionTime.end(), now);
  std::fill(other.m_buttonDebounceTransitionTime.begin(), other.m_buttonDebounceTransitionTime.end(), now);
}

double XboxController::GetX(JoystickHand hand) const {
  return GetRawAxis(static_cast<int>(hand == JoystickHand::kLeftHand ? Axis::kLeftX : Axis::kRightX));
}

double XboxController::GetY(JoystickHand hand) const {
  return GetRawAxis(static_cast<int>(hand == JoystickHand::kLeftHand ? Axis::kLeftY : Axis::kRightY));
}

double XboxController::GetTriggerAxis(JoystickHand hand) const {
  return GetRawAxis(static_cast<int>(hand == JoystickHand::kLeftHand ? Axis::kLeftTrigger : Axis::kRightTrigger));
}

bool XboxController::GetDebouncedButton(Button buttonIdx) {
  return UpdateButton(buttonIdx).debounceActive;
}

bool XboxController::GetDebouncedButtonPressed(Button buttonIdx) {
  return UpdateButton(buttonIdx).debouncePress;
}

bool XboxController::GetDebouncedButtonReleased(Button buttonIdx) {
  return UpdateButton(buttonIdx).debounceRelease;
}

bool XboxController::GetDebouncedButton(std::initializer_list<Button> buttonCombo) {
  std::vector<UpdateStatus> updates;
  updates.reserve(buttonCombo.size());
  std::transform(buttonCombo.begin(), buttonCombo.end(), std::back_inserter(updates), [this](Button buttonIdx) {
    return UpdateButton(buttonIdx);
  });

  return std::all_of(updates.begin(), updates.end(), [](UpdateStatus newState) { return newState.debounceActive; });
}

bool XboxController::GetDebouncedButtonPressed(std::initializer_list<Button> buttonCombo) {
  std::vector<UpdateStatus> updates;
  updates.reserve(buttonCombo.size());
  std::transform(buttonCombo.begin(), buttonCombo.end(), std::back_inserter(updates), [this](Button buttonIdx) {
    return UpdateButton(buttonIdx);
  });

  return std::all_of(updates.begin(), updates.end(), [](UpdateStatus newState) { return newState.debounceActive; }) &&
         std::any_of(updates.begin(), updates.end(), [](UpdateStatus newState) { return newState.debouncePress; });
}

bool XboxController::GetDebouncedButtonReleased(std::initializer_list<Button> buttonCombo) {
  std::vector<UpdateStatus> updates;
  updates.reserve(buttonCombo.size());
  std::transform(buttonCombo.begin(), buttonCombo.end(), std::back_inserter(updates), [this](Button buttonIdx) {
    return UpdateButton(buttonIdx);
  });

  return std::none_of(updates.begin(), updates.end(), [](UpdateStatus newState) { return newState.debounceActive; }) &&
         std::any_of(updates.begin(), updates.end(), [](UpdateStatus newState) { return newState.debounceRelease; });
}

bool XboxController::GetRawButton(Button buttonIdx) {
  return UpdateButton(buttonIdx).rawActive;
}

bool XboxController::GetRawButtonPressed(Button buttonIdx) {
  return UpdateButton(buttonIdx).pressed;
}

bool XboxController::GetRawButtonReleased(Button buttonIdx) {
  return UpdateButton(buttonIdx).released;
}

bool XboxController::GetRawButton(std::initializer_list<Button> buttonCombo) {
  std::vector<UpdateStatus> updates;
  updates.reserve(buttonCombo.size());
  std::transform(buttonCombo.begin(), buttonCombo.end(), std::back_inserter(updates), [this](Button buttonIdx) {
    return UpdateButton(buttonIdx);
  });

  return std::all_of(updates.begin(), updates.end(), [](UpdateStatus newState) { return newState.rawActive; });
}

bool XboxController::GetRawButtonPressed(std::initializer_list<Button> buttonCombo) {
  std::vector<UpdateStatus> updates;
  updates.reserve(buttonCombo.size());
  std::transform(buttonCombo.begin(), buttonCombo.end(), std::back_inserter(updates), [this](Button buttonIdx) {
    return UpdateButton(buttonIdx);
  });

  return std::all_of(updates.begin(), updates.end(), [](UpdateStatus newState) { return newState.rawActive; }) &&
         std::any_of(updates.begin(), updates.end(), [](UpdateStatus newState) { return newState.pressed; });
}

bool XboxController::GetRawButtonReleased(std::initializer_list<Button> buttonCombo) {
  std::vector<UpdateStatus> updates;
  updates.reserve(buttonCombo.size());
  std::transform(buttonCombo.begin(), buttonCombo.end(), std::back_inserter(updates), [this](Button buttonIdx) {
    return UpdateButton(buttonIdx);
  });

  return std::none_of(updates.begin(), updates.end(), [](UpdateStatus newState) { return newState.rawActive; }) &&
         std::any_of(updates.begin(), updates.end(), [](UpdateStatus newState) { return newState.released; });
}

VibrationModel XboxController::GetVibration() const {
  return m_vibrationModel;
}

void XboxController::SetVibration(VibrationModel newVibrationModel) {
  m_vibrationModel = newVibrationModel;
  UpdateVibration();
}

void XboxController::UpdateVibration() {
  const auto vibrationStatus = m_vibrationModel();
  SetRumble(kLeftRumble, vibrationStatus.intensityLeft);
  SetRumble(kRightRumble, vibrationStatus.intensityRight);
}

XboxController::UpdateStatus XboxController::UpdateButton(Button buttonIdx) {
  UpdateStatus retVal;

  // Limit annoying joystick not connected errors
  if (!IsConnected()) {
    return retVal;
  }

  bool newVal;
  switch (buttonIdx) {
    case Button::kA:
    case Button::kB:
    case Button::kX:
    case Button::kY:
    case Button::kBumperLeft:
    case Button::kBumperRight:
    case Button::kBack:
    case Button::kStart:
    case Button::kStickLeft:
    case Button::kStickRight:
      newVal = frc::GenericHID::GetRawButton(static_cast<int>(buttonIdx));
      break;
    case Button::kLeftTrigger:
      newVal = GetTriggerAxis(JoystickHand::kLeftHand) > analogTriggerThresh;
      break;
    case Button::kRightTrigger:
      newVal = GetTriggerAxis(JoystickHand::kRightHand) > analogTriggerThresh;
      break;
    case Button::kUp:
      newVal = GetPOVButtons().up;
      break;
    case Button::kRight:
      newVal = GetPOVButtons().right;
      break;
    case Button::kDown:
      newVal = GetPOVButtons().down;
      break;
    case Button::kLeft:
      newVal = GetPOVButtons().left;
      break;
    case Button::COUNT:
    default:
      throw std::runtime_error("Invalid button index selected");
  }

  const bool prevRawVal = m_rawButtonStatus.at(static_cast<int>(buttonIdx));
  const bool activeDebounceVal = m_buttonDebounceStatus.at(static_cast<int>(buttonIdx));
  const auto curTime = std::chrono::steady_clock::now();

  // Record time when first new button state was detected so debounce has reference time
  if (prevRawVal == activeDebounceVal && newVal != activeDebounceVal) {
    m_buttonDebounceTransitionTime.at(static_cast<int>(buttonIdx)) = curTime;
  }

  const auto activeTransitionTime = m_buttonDebounceTransitionTime.at(static_cast<int>(buttonIdx));

  retVal.pressed = newVal && !prevRawVal;
  retVal.released = !newVal && prevRawVal;

  if (newVal != activeDebounceVal) {
    const auto timeSinceTransition = units::millisecond_t{static_cast<double>(
        std::chrono::duration_cast<std::chrono::milliseconds>(curTime - activeTransitionTime).count())};
    if (newVal) {
      if (timeSinceTransition >= m_buttonDebounceSettings.at(static_cast<int>(buttonIdx)).activateTime) {
        retVal.debouncePress = true;
        m_buttonDebounceStatus.at(static_cast<int>(buttonIdx)) = newVal;
      }
    } else {
      if (timeSinceTransition >= m_buttonDebounceSettings.at(static_cast<int>(buttonIdx)).clearTime) {
        retVal.debounceRelease = true;
        m_buttonDebounceStatus.at(static_cast<int>(buttonIdx)) = newVal;
      }
    }
  }

  retVal.rawActive = newVal;
  retVal.debounceActive = m_buttonDebounceStatus.at(static_cast<int>(buttonIdx));

  m_rawButtonStatus.at(static_cast<int>(buttonIdx)) = newVal;

  return retVal;
}

XboxController::DPadButtons XboxController::GetPOVButtons() {
  const auto povAngle{GetPOV()};
  return DPadButtons{(povAngle >= 0 && povAngle <= 45) || povAngle >= 315,
                     povAngle >= 45 && povAngle <= 135,
                     povAngle >= 135 && povAngle <= 225,
                     povAngle >= 225 && povAngle <= 315};
}
