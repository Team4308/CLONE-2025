package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class EndeffectorSubsystem extends SubsystemBase {

    private TalonFX IntakeMotor = new TalonFX(Ports.Endeffector.IntakeMotor);
    private TalonFX CenteringMotor = new TalonFX(Ports.Endeffector.CenteringMotor);

    public EndeffectorSubsystem() {
        
    }



}
