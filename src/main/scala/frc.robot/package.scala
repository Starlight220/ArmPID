package frc

import edu.wpi.first.networktables.NetworkTableEntry

package object robot {
  implicit def Entry2Double(entry: NetworkTableEntry) = entry.getDouble(0.0)
  implicit def Entry2Boolean(entry: NetworkTableEntry) = entry.getBoolean(false)
}
