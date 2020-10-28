package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import frc.robot.ArmSystem.Simulation._
import frc.robot.ArmSystem._
import frc.robot.PidUpdater._


/**
 * This is a sample program to demonstrate the use of elevator simulation with existing code.
 */
class Robot extends TimedRobot {
  override def robotInit() {
    ArmSystem.Simulation
    PidUpdater.values
  }

  override def simulationPeriodic() {
    if(isOperatorControlEnabled) runSimulation()
  }


  override def teleopPeriodic() {
    updatePid(values)
    applyPidOutput(activeEntry, setpointEntry)
  }

  // This just makes sure that our simulation code knows that the motor's off.
  override def disabledInit(): Unit = neutral
}
