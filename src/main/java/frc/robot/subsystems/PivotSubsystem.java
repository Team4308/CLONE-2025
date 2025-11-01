package frc.robot.subsystems;

import java.security.PublicKey;

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

import ca.team4308.absolutelib.math.DoubleUtils;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;

public class PivotSubsystem extends SubsystemBase {
    public TalonFX m_pivotMotor = new TalonFX(Ports.Pivot.PivotMotor);
    private CANcoder m_pivotEncoder = new CANcoder(Ports.Pivot.PivotEncoder);
    private DigitalInput topBreak = new DigitalInput(Ports.Pivot.TopLimitSwitch);
    private DigitalInput botBreak = new DigitalInput(Ports.Pivot.BotLimitSwitch);

    private boolean atPosition = false;
    private double targetAngle = 128;
    private double encoderOffset = 0;

    private ArmFeedforward feedforward = new ArmFeedforward(0, 0.28, 0.0155, 0);
    private ProfiledPIDController pidController = new ProfiledPIDController(0.06, 0.0, 0.0,
            new TrapezoidProfile.Constraints(360, 720));

    public PivotSubsystem() {
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        encoderOffset = 107;

        m_pivotMotor.getConfigurator().apply(talonFXConfigs);
    }

    public void setPivotTarget(double angle) {
        targetAngle = DoubleUtils.clamp(angle, -10, 128);
    }

    public double getPivotAngle() {
        return m_pivotEncoder.getPosition().getValueAsDouble() * 360 * 12 / 22 + encoderOffset;
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
        double currentAngle = getPivotAngle();

        if (topBreak.get()) {
            // encoderOffset = m_pivotEncoder.getPosition().getValueAsDouble() - 130;
        } else if (botBreak.get()) {
            // encoderOffset = m_pivotEncoder.getPosition().getValueAsDouble() - 5;
        }

        atPosition = Math.abs(currentAngle - targetAngle) < 3;

        double pidOutput = pidController.calculate(currentAngle, targetAngle);
        double feedforwardVolts = feedforward.calculate(Units.degreesToRadians(targetAngle),
                pidController.getSetpoint().velocity);

        double outputVolts = feedforwardVolts + pidOutput;

        driveVoltage(outputVolts);

        Logger.recordOutput("Subsystems/Pivot/AtTArget", atPosition);
        Logger.recordOutput("Subsystems/Pivot/TargetPos", pidController.getSetpoint().position);
        Logger.recordOutput("Subsystems/Pivot/CurPos", currentAngle);
        Logger.recordOutput("Subsystems/Pivot/Voltage", m_pivotMotor.getMotorVoltage().getValueAsDouble());
    }
}
