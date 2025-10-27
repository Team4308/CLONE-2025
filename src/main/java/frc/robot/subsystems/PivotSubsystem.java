package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class PivotSubsystem extends SubsystemBase {
    public TalonFX m_pivotMotor = new TalonFX(Ports.Pivot.PivotMotor);
    public CANcoder m_pivotEncoder = new CANcoder(Ports.Pivot.PivotEncoder);

    private MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    private boolean atPosition = false;
    private double targetRotations = 0;

    public PivotSubsystem() {
        var talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.0; // output to overcome static friction (output)
        slot0Configs.kV = 0.0; // output per unit of target velocity (output/rps)
        slot0Configs.kP = 0.0; // output per unit of error in position (output/rotation)
        slot0Configs.kI = 0.0; // output per unit of integrated error in position (output/(rotation*s))
        slot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
        slot0Configs.kG = 0.0; // output to overcome gravity (output)
        slot0Configs.kA = 0.0; // output per unit of target acceleration (output/(rps/s))
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine; // Arm feedforward for talonFX, 0 is horizontal and 1 is
                                                                // up (of the sensor)

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        // set netural mode
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.Feedback.FeedbackRemoteSensorID = m_pivotEncoder.getDeviceID();
        talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        talonFXConfigs.Feedback.RotorToSensorRatio = 0.0;
        talonFXConfigs.Feedback.SensorToMechanismRatio = 0.0;

        m_pivotMotor.getConfigurator().apply(talonFXConfigs);

        CANcoderConfiguration canConfig = new CANcoderConfiguration();
        canConfig.MagnetSensor.MagnetOffset = 0.0;
        canConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        m_pivotEncoder.getConfigurator().apply(canConfig);
    }

    public void setPivotAngle(double angle) {
        /*
         * TalonFX uses a setpoint of mechanism rotations not degrees
         * 
         * TO DO: fix math
         */

        double goalRad = Units.degreesToRadians(angle); // more math needs to be done

        targetRotations = goalRad;
    }

    public double getPivotAngle() {
        /*
         * the sensor is set such at 0 is horizontal (mechanism), and 1 is straight up
         * 
         * TO DO: fix math
         */
        double rawAngle = m_pivotEncoder.getPosition().getValueAsDouble();
        double angle = ((rawAngle / Constants.Pivot.Pivot_EncoderTicksPerRevolution) * 360.0)
                / Constants.Pivot.Gear_Ratio;
        return angle;
    }

    @Override
    public void periodic() {
        double currentRad = Units.degreesToRadians(getPivotAngle());

        atPosition = currentRad - targetRotations < Constants.Pivot.ROTATION_TOLERANCE;

        if (!atPosition) {
            m_pivotMotor.setControl(m_request.withPosition(targetRotations));
        }

        Logger.recordOutput("/Subsystems/Pivot/TargetPos", targetRotations);
        Logger.recordOutput("/Subsystems/Pivot/Voltage", m_pivotMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("/Subsystmes/Pivot/CurPos", getPivotAngle());
    }
}
