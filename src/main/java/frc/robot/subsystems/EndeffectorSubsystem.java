package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;

public class EndEffectorSubsystem extends SubsystemBase {

    public TalonFX IntakeMotor = new TalonFX(Ports.EndEffector.IntakeMotor);
    public TalonFX CenteringMotor = new TalonFX(Ports.EndEffector.CenteringMotor);
    private DigitalInput leftBeam = new DigitalInput(Ports.EndEffector.leftDIO);
    private DigitalInput rightBeam = new DigitalInput(Ports.EndEffector.rightDIO);
    public boolean isIntaking = false;

    public EndEffectorSubsystem() {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = 0.1; // Add 0.1 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative
        IntakeMotor.getConfigurator().apply(slot0Configs);
        CenteringMotor.getConfigurator().apply(slot0Configs);
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

    public void CenterCoral(int dir) {
        CenteringMotor.set(Constants.EndEffector.CenteringSpeed * dir);
        IntakeMotor.set(Constants.EndEffector.IntakeSpeed / 5);
    }

    public void StopCentering() {
        CenteringMotor.set(0);
        IntakeMotor.set(0);
    }

    public void StopMotors() {
        IntakeMotor.set(0);
        CenteringMotor.set(0);
    }

    public void setMotorSpeed(double speed) {
        IntakeMotor.set(speed);
    }

    @Override
    public void periodic() {
        if (getIntaken()) {
            isIntaking = false;
            IntakeMotor.set(0);
            StopCentering();
        }

        if (leftBeam.get() ^ rightBeam.get()) {
            if (leftBeam.get()) {
                CenterCoral(1);
            } else {
                CenterCoral(-1);
            }
        }

        Logger.recordOutput("Subsystems/EndEffector/LeftBeam", (boolean) leftBeam.get());
        Logger.recordOutput("Subsystems/EndEffector/RightBeam", (boolean) rightBeam.get());
        Logger.recordOutput("Subsystems/EndEffector/Intaken", getIntaken());
        Logger.recordOutput("Subsystems/EndEffector/IntakeVoltage", IntakeMotor.getMotorVoltage().getValueAsDouble());
    }

}
