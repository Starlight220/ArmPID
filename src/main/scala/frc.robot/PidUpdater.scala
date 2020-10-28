package frc.robot

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

object PidUpdater {
  val activeEntry: NetworkTableEntry = SmartDashboard.getEntry("active")
  activeEntry.setBoolean(false)
  val setpointEntry: NetworkTableEntry = SmartDashboard.getEntry("setpoint")
  setpointEntry.setDouble(0.0)
  val kpEntry: NetworkTableEntry = SmartDashboard.getEntry("kp")
  kpEntry.setDouble(0.0)
  val kiEntry: NetworkTableEntry = SmartDashboard.getEntry("ki")
  kiEntry.setDouble(0.0)
  val kdEntry: NetworkTableEntry = SmartDashboard.getEntry("kd")
  kdEntry.setDouble(0.0)


  def values: (Double, Double, Double) = (kpEntry, kiEntry, kdEntry)

}
