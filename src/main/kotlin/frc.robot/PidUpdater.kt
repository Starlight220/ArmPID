package frc.robot

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableType
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.reflect.KProperty

object PidUpdater {
    private val activeEntry: NetworkTableEntry = SmartDashboard.getEntry("active")
    private val setpointEntry: NetworkTableEntry = SmartDashboard.getEntry("setpoint")
    private val kpEntry: NetworkTableEntry = SmartDashboard.getEntry("kp")
    private val kiEntry: NetworkTableEntry = SmartDashboard.getEntry("ki")
    private val kdEntry: NetworkTableEntry = SmartDashboard.getEntry("kd")

    init {
        activeEntry.setBoolean(false)
        setpointEntry.setDouble(0.0)
        kpEntry.setDouble(0.0)
        kiEntry.setDouble(0.0)
        kdEntry.setDouble(0.0)
    }

    val setpoint: Double get() = setpointEntry.getDouble(0.0)
    val active: Boolean get() = activeEntry.getBoolean(false)

    val kp: Double get() = kpEntry.getDouble(0.0)
    val ki: Double get() = kiEntry.getDouble(0.0)
    val kd: Double get() = kdEntry.getDouble(0.0)
}
