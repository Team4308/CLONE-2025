package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Constants.Swerve.Auton.Angle;

public class SlapdownSubsystem extends SubsystemBase {
    public TalonFX PivotMotor = new TalonFX(Ports.Slapdown.PivotMotor);
    public CANcoder PivotEncoder = new CANcoder(Ports.Slapdown.PivotEncoder);

    public SlapdownSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = Constants.Slapdown.Pivot_kP;
        config.Slot0.kI = Constants.Slapdown.Pivot_kI;
        config.Slot0.kD = Constants.Slapdown.Pivot_kD;
        PivotMotor.getConfigurator().apply(config);
        PivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setPivotAngle(double angle) {
        // use FF and PID to set the angle of the slapdown

    }

    public double getPivotAngle() {
        double rawAngle = PivotEncoder.getPosition().getValueAsDouble();
        double angle = ((rawAngle / Constants.Slapdown.Pivot_EncoderTicksPerRevolution) * 360.0 ) / Constants.Slapdown.Gear_Ratio;
        return angle;
    }

    @Override
    public void periodic() {
    }
}
