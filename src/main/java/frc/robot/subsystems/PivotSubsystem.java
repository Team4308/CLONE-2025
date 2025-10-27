package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;

public class PivotSubsystem extends SubsystemBase {
    public TalonFX PivotMotor = new TalonFX(Ports.Slapdown.PivotMotor);
    public CANcoder PivotEncoder = new CANcoder(Ports.Slapdown.PivotEncoder);

    private final ProfiledPIDController pivotPID = new ProfiledPIDController(
        Constants.Slapdown.Pivot_kP,
        Constants.Slapdown.Pivot_kI,
        Constants.Slapdown.Pivot_kD,
        new TrapezoidProfile.Constraints(
            Constants.Slapdown.PivotMaxVelRadPerSec,
            Constants.Slapdown.PivotMaxAccelRadPerSec2
        )
    );
    private final ArmFeedforward pivotFF = new ArmFeedforward(
        Constants.Slapdown.Pivot_kS,
        Constants.Slapdown.Pivot_kG,
        Constants.Slapdown.Pivot_kV,
        Constants.Slapdown.Pivot_kA
    );
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    private boolean hasGoal = false;

    public PivotSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = Constants.Slapdown.Pivot_kP;
        config.Slot0.kI = Constants.Slapdown.Pivot_kI;
        config.Slot0.kD = Constants.Slapdown.Pivot_kD;
        PivotMotor.getConfigurator().apply(config);
        PivotMotor.setNeutralMode(NeutralModeValue.Brake);
        pivotPID.setTolerance(
            Units.degreesToRadians(Constants.Slapdown.PivotPositionToleranceDeg),
            Units.degreesToRadians(Constants.Slapdown.PivotVelocityToleranceDegPerSec)
        );
    }

    public void setPivotAngle(double angle) {
        double currentRad = Units.degreesToRadians(getPivotAngle());
        double goalRad = Units.degreesToRadians(angle);

        if (!hasGoal) {
            pivotPID.reset(currentRad);
            hasGoal = true;
        }
        pivotPID.setGoal(goalRad);
    }

    public double getPivotAngle() {
        double rawAngle = PivotEncoder.getPosition().getValueAsDouble();
        double angle = ((rawAngle / Constants.Slapdown.Pivot_EncoderTicksPerRevolution) * 360.0)
                / Constants.Slapdown.Gear_Ratio;
        return angle;
    }
 
    public double getEncoder() {
        return PivotEncoder.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        if (!hasGoal) {
            return;
        }

        double currentRad = Units.degreesToRadians(getPivotAngle());

        double pidVolts = pivotPID.calculate(currentRad);

        TrapezoidProfile.State sp = pivotPID.getSetpoint();
        double ffVolts = pivotFF.calculate(sp.position, sp.velocity);

        double outVolts = MathUtil.clamp(pidVolts + ffVolts, -12.0, 12.0);
        PivotMotor.setControl(voltageRequest.withOutput(outVolts));
    }
}
