package frc.robot

import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.PWMVictorSPX
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.EncoderSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.system.plant.DCMotor
import edu.wpi.first.wpilibj.util.Units
import edu.wpi.first.wpiutil.math.VecBuilder

object ArmSystem {
    private const val kMotorPort = 0
    private const val kEncoderAChannel = 0
    private const val kEncoderBChannel = 1
    private const val kJoystickPort = 0

    // The P gain for the PID controller that drives this arm.
    private const val kArmKp = 5.0

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    //  = (2 * PI rads) / (4096 pulses)
    private const val kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096

    // The arm gearbox represents a gerbox containing two Vex 775pro motors.
    private val m_armGearbox = DCMotor.getVex775Pro(2)

    // Standard classes for controlling our elevator
    private val m_controller = PIDController(kArmKp, 0.0, 0.0)
    private val m_encoder = Encoder(kEncoderAChannel, kEncoderBChannel)
    private val m_motor = PWMVictorSPX(kMotorPort)

    // Simulation classes help us simulate what's going on, including gravity.
    private const val m_armReduction = 100.0
    private const val m_armMass = 5.0 // Kilograms
    private val m_armLength = Units.inchesToMeters(30.0)

    init {
        m_encoder.distancePerPulse = kArmEncoderDistPerPulse
    }

    // This arm sim represents an arm that can travel from -180 degrees (rotated straight backwards)
    // to 0 degrees (rotated straight forwards).
    private val m_armSim = SingleJointedArmSim(m_armGearbox,
            m_armReduction,
            SingleJointedArmSim.estimateMOI(m_armLength, m_armMass),
            m_armLength,
            Units.degreesToRadians(-180.0),
            Units.degreesToRadians(0.0),
            m_armMass,
            true,
            VecBuilder.fill(Units.degreesToRadians(0.5)) // Add noise with a std-dev of 0.5 degrees
    )
    private val m_encoderSim = EncoderSim(m_encoder)


    fun simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage())

        // Next, we update it. The standard loop time is 20ms.
        m_armSim.update(0.020)

        // Finally, we set our simulated encoder's readings and simulated battery voltage
        m_encoderSim.distance = Units.radiansToDegrees(m_armSim.angleRads)
        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.currentDrawAmps))
    }

    fun neutral() = m_motor.set(0.0)

    fun applyOutput(active: Boolean, setpoint: Double) {
        if (active) {
            // Here, we run PID control like normal, with a constant setpo  of 30in.
            val pidOutput = m_controller.calculate(m_encoder.distance, setpoint)
            m_motor.setVoltage(pidOutput)
        } else {
            // Otherwise, we disable the motor.
            neutral()
        }
    }

    fun updatePID(kp:Double, ki:Double, kd:Double) = m_controller.setPID(kp,ki,kd)
}