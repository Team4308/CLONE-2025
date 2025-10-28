package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;

public class EndEffectorSubsystem extends SubsystemBase {

    private TalonFX IntakeMotor = new TalonFX(Ports.EndEffector.IntakeMotor);
    private TalonFX CenteringMotor = new TalonFX(Ports.EndEffector.CenteringMotor);
    private DigitalInput leftBeam = new DigitalInput(Ports.EndEffector.leftDIO);
    private DigitalInput rightBeam = new DigitalInput(Ports.EndEffector.rightDIO);
    public boolean isIntaking = false;

    public EndEffectorSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = Constants.EndEffector.kP;
        config.Slot0.kI = Constants.EndEffector.kI;
        config.Slot0.kD = Constants.EndEffector.kD;
        IntakeMotor.getConfigurator().apply(config);
        CenteringMotor.getConfigurator().apply(config);
    }

    public boolean simIntaking = false;

    public boolean getIntaken() {
        if (Robot.isSimulation()) {
            return simIntaking;
        }
        return leftBeam.get() || rightBeam.get();
    }

    public void Intake() {
        if (!getIntaken()) {
            IntakeMotor.set(Constants.EndEffector.IntakeSpeed);
            isIntaking = true;
        }
    }

    public void Score() {
        if (getIntaken()) {
            IntakeMotor.set(-Constants.EndEffector.ScoreSpeed);
        }
    }

    public void CenterCoral() {
        CenteringMotor.set(Constants.EndEffector.CenteringSpeed);
    }

    public void StopCentering() {
        CenteringMotor.set(0);
    }

    public void StopMotors() {
        IntakeMotor.set(0);
        CenteringMotor.set(0);
    }

    @Override
    public void periodic() {
        // Stop intaking once the game piece is detected
        if (isIntaking && getIntaken()) {
            IntakeMotor.set(0);
            isIntaking = false;
        }

        Logger.recordOutput("Subsystems/EndEffector/LeftBeam", (boolean) leftBeam.get());
        Logger.recordOutput("Subsystems/EndEffector/RightBeam", (boolean) rightBeam.get());
        Logger.recordOutput("Subsystems/EndEffector/Intaken", getIntaken());
        Logger.recordOutput("Subsystems/EndEffector/LeftBeam", IntakeMotor.getMotorVoltage().getValueAsDouble());
    }

}
