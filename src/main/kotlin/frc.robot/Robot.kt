package frc.robot

import edu.wpi.first.wpilibj.TimedRobot

class Robot : TimedRobot() {
    override fun robotInit() {
        PidUpdater
        ArmSystem
    }

    override fun simulationPeriodic() = ArmSystem.simulationPeriodic()

    override fun teleopPeriodic() {
        PidUpdater.apply {
            ArmSystem.applyOutput(active, setpoint)
            ArmSystem.updatePID(kp, ki, kd)
        }

    }

    override fun disabledInit() {
        // This just makes sure that our simulation code knows that the motor's off.
        ArmSystem.neutral()
    }

}