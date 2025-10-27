package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class ClimbSubsystem extends SubsystemBase {

    public TalonFX ClimbMotor = new TalonFX(Ports.Climb.Motor);

    public ClimbSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // Like Add later or somthing
        ClimbMotor.getConfigurator().apply(config);
    }


    // Idk change this to have logic later
    public void StartClimb() {
        ClimbMotor.set(0.5);
    }

    public void StopClimb() {
        ClimbMotor.set(0);
    }


}
