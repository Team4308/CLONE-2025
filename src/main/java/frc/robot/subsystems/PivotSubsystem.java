package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.ColorSensorV3.RawColor;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;

public class PivotSubsystem extends SubsystemBase {
    public TalonFX m_pivotMotor = new TalonFX(Ports.Pivot.PivotMotor);
    public CANcoder m_pivotEncoder = new CANcoder(Ports.Pivot.PivotEncoder);

    private MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    private boolean atPosition = false;
    private double targetRotations = 0;

    public TalonFXSimState m_pivotMotorSim;
    public CANcoderSimState m_pivotEncoderSim;

    public PivotSubsystem() {
        var talonFXConfigs = new TalonFXConfiguration();
        m_pivotMotor.setInverted(true);
        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.0; // output to overcome static friction (output)
        slot0Configs.kV = 6.7; // output per unit of target velocity (output/rps)
        slot0Configs.kP = 60; // output per unit of error in position (output/rotation)
        slot0Configs.kI = 0; // output per unit of integrated error in position (output/(rotation*s))
        slot0Configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
        slot0Configs.kG = 2.37; // output to overcome gravity (output)
        slot0Configs.kA = 0.07; // output per unit of target acceleration (output/(rps/s))
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine; // Arm feedforward for talonFX, 0 is horizontal and 1 is
                                                                // up (of the sensor)

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        // set netural mode
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        talonFXConfigs.Feedback.FeedbackRemoteSensorID = m_pivotEncoder.getDeviceID();
        talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // talonFXConfigs.Feedback.RotorToSensorRatio = 0.08081; // Rotor range must be
        // 1:
        // talonFXConfigs.Feedback.SensorToMechanismRatio = 90; // Cancoder range must
        // be 0-1 for mechanism range
        // 10-140 respectively, however the mechanism is
        // geared down by 1.83x first

        m_pivotMotor.getConfigurator().apply(talonFXConfigs);

        CANcoderConfiguration canConfig = new CANcoderConfiguration();
        canConfig.MagnetSensor.MagnetOffset = 0.0; // Set this on robot turn on? or a button to reset this
        canConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        m_pivotEncoder.getConfigurator().apply(canConfig);

        m_pivotMotorSim = m_pivotMotor.getSimState();
        m_pivotEncoderSim = m_pivotEncoder.getSimState();
    }

    public void setPivotTarget(double angle) {
        /*
         * TalonFX uses a setpoint of mechanism rotations not degrees
         */

        targetRotations = angle / 90;
    }

    public double simEncoderVal = 0;

    public double getPivotAngle() {
        /*
         * the sensor is set such at 0 is horizontal (mechanism), and 1 is straight up
         * 
         * 
         */

        if (Robot.isSimulation()) {
            m_pivotEncoder.setPosition(simEncoderVal, 0.001);
        }

        double rawAngle = m_pivotEncoder.getPosition().getValueAsDouble();

        return rawAngle * 90;
    }

    public double getPivotVoltage() {
        return m_pivotMotor.getMotorVoltage().getValueAsDouble();
    }

    public void driveVoltage(double volts) {
        m_pivotMotor.setVoltage(volts);
    }

    public boolean atPosition() {
        return atPosition;
    }

    @Override
    public void periodic() {
        double currentRotations = getPivotAngle() / 90;

        atPosition = Math.abs(currentRotations - targetRotations) < Constants.Pivot.ROTATION_TOLERANCE;
        m_pivotMotor.setControl(m_request.withPosition(targetRotations));

        Logger.recordOutput("Subsystems/Pivot/AtTArget", atPosition);
        Logger.recordOutput("Subsystems/Pivot/TargetPos", targetRotations);
        Logger.recordOutput("Subsystems/Pivot/CurPos", currentRotations);
        Logger.recordOutput("Subsystems/Pivot/Degrees", currentRotations * 90);
        Logger.recordOutput("Subsystems/Pivot/Voltage", m_pivotMotor.getMotorVoltage().getValueAsDouble());
    }
}
