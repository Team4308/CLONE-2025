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

public class PivotSubsystem extends SubsystemBase {
    public TalonFX PivotMotor = new TalonFX(Ports.Pivot.PivotMotor);
    public CANcoder PivotEncoder = new CANcoder(Ports.Pivot.PivotEncoder);

    private final ProfiledPIDController pivotPID = new ProfiledPIDController(
        Constants.Pivot.Pivot_kP,
        Constants.Pivot.Pivot_kI,
        Constants.Pivot.Pivot_kD,
        new TrapezoidProfile.Constraints(
            Constants.Pivot.PivotMaxVelRadPerSec,
            Constants.Pivot.PivotMaxAccelRadPerSec2
        )
    );
    private final ArmFeedforward pivotFF = new ArmFeedforward(
        Constants.Pivot.Pivot_kS,
        Constants.Pivot.Pivot_kG,
        Constants.Pivot.Pivot_kV,
        Constants.Pivot.Pivot_kA
    );
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);
    private boolean hasGoal = false;

    public PivotSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = Constants.Pivot.Pivot_kP;
        config.Slot0.kI = Constants.Pivot.Pivot_kI;
        config.Slot0.kD = Constants.Pivot.Pivot_kD;
        PivotMotor.getConfigurator().apply(config);
        PivotMotor.setNeutralMode(NeutralModeValue.Brake);
        pivotPID.setTolerance(
            Units.degreesToRadians(Constants.Pivot.PivotPositionToleranceDeg),
            Units.degreesToRadians(Constants.Pivot.PivotVelocityToleranceDegPerSec)
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
        double angle = ((rawAngle / Constants.Pivot.Pivot_EncoderTicksPerRevolution) * 360.0)
                / Constants.Pivot.Gear_Ratio;
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
