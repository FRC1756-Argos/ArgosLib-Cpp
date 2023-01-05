/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/general/nt_subscriber.h"

#include <networktables/NetworkTableInstance.h>

using argos_lib::NTSubscriber;

NTSubscriber::NTSubscriber(const std::string& tableName)
    : m_pntTable{nt::NetworkTableInstance::GetDefault().GetTable(tableName)} {}

void NTSubscriber::AddMonitor(const std::string& keyName,
                              std::function<void(double)> onUpdateCallback,
                              const double defaultValue,
                              const bool forceUpdate) {
  if (forceUpdate) {
    m_pntTable->PutNumber(keyName, defaultValue);
  } else {
    m_pntTable->SetDefaultNumber(keyName, defaultValue);
  }
  m_pntTable->AddEntryListener(
      keyName,
      [onUpdateCallback](nt::NetworkTable* table,
                         std::string_view key,
                         nt::NetworkTableEntry entry,
                         std::shared_ptr<nt::Value> value,
                         int flags) { onUpdateCallback(value->GetDouble()); },
      NT_NOTIFY_UPDATE);
}
