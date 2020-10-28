package frc.robot

import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.simulation.{BatterySim, EncoderSim, RoboRioSim, SingleJointedArmSim}
import edu.wpi.first.wpilibj.system.plant.DCMotor
import edu.wpi.first.wpilibj.util.Units
import edu.wpi.first.wpilibj.{Encoder, PWMVictorSPX, RobotBase, RobotController}
import edu.wpi.first.wpiutil.math.VecBuilder

import scala.math.Pi

object ArmSystem {
  // Standard classes for controlling our elevator
  val m_controller: PIDController = new PIDController(0.0, 0, 0)
  val m_encoder: Encoder = new Encoder(0, 1)
  val m_motor: PWMVictorSPX = new PWMVictorSPX(0)

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
  val kArmEncoderDistPerPulse: Double = 2.0 * Pi / 4096.0
  m_encoder.setDistancePerPulse(kArmEncoderDistPerPulse)

  def applyPidOutput(active: Boolean, setpoint: Double): Unit = {
    if (active) {
      // Here, we run PID control like normal, with a constant setpoint of 30in.
      val pidOutput = m_controller.calculate(m_encoder.getDistance, setpoint)
      m_motor.setVoltage(pidOutput)
    } else neutral
      // Otherwise, we disable the motor.
  }
  def neutral: Unit = m_motor.set(0.0)

  def updatePid(values: (Double, Double, Double)) = {
    val (kp, ki, kd) = values
    m_controller.setPID(kp, ki, kd)
  }

  object Simulation {
    // The arm gearbox represents a gerbox containing two Vex 775pro motors.
    val m_armGearbox: DCMotor = DCMotor.getVex775Pro(2)
    // Simulation classes help us simulate what's going on, including gravity.
    val m_armReduction = 100
    val m_armMass = 5.0 // Kilograms
    val m_armLength: Double = Units.inchesToMeters(30)
    // This arm sim represents an arm that can travel from -180 degrees (rotated straight backwards)
    // to 0 degrees (rotated straight forwards).
    val m_armSim: SingleJointedArmSim = new SingleJointedArmSim(m_armGearbox,
      m_armReduction,
      SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
      m_armLength,
      Units.degreesToRadians(180),
      Units.degreesToRadians(0),
      m_armMass,
      true,
      VecBuilder.fill(Units.degreesToRadians(0.5)) // Add noise with a std-dev of 0.5 degrees
    )
    val m_encoderSim: EncoderSim = new EncoderSim(m_encoder)

    def runSimulation(): Unit = {
      // In this method, we update our simulation of what our elevator is doing
      // First, we set our "inputs" (voltages)
      m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage)

      // Next, we update it. The standard loop time is 20ms.
      m_armSim.update(0.020)

      // vally, we set our simulated encoder's readings and simulated battery voltage
      m_encoderSim.setDistance(m_armSim.getAngleRads)
      // SimBattery estimates loaded battery voltages
      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps))
    }
  }

}
