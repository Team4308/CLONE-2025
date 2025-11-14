package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.Robot;

public class EndEffectorSubsystem extends SubsystemBase {

    public TalonFX IntakeMotor = new TalonFX(Ports.EndEffector.IntakeMotor);
    public TalonFX CenteringMotor = new TalonFX(Ports.EndEffector.CenteringMotor);
    
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


    public void CenterCoral(int dir) {
        CenteringMotor.set(Constants.EndEffector.CenteringSpeed * dir);
    }

    public void setSpeed(double s) {
        IntakeMotor.set(s);
    }

    public void StopCentering() {
        CenteringMotor.set(0);
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

        Logger.recordOutput("Subsystems/EndEffector/IntakeVoltage", IntakeMotor.getMotorVoltage().getValueAsDouble());
    }

}
