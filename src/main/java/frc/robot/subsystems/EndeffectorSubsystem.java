package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class EndeffectorSubsystem extends SubsystemBase {

    private TalonFX IntakeMotor = new TalonFX(Ports.EndEffector.IntakeMotor);
    private TalonFX CenteringMotor = new TalonFX(Ports.EndEffector.CenteringMotor);
    private DigitalInput leftBeam = new DigitalInput(Ports.EndEffector.leftBeam);
    private DigitalInput rightBeam = new DigitalInput(Ports.EndEffector.rightBeam);

    public EndeffectorSubsystem() {

    }

    boolean getIntaken() {
        return leftBeam.get() || rightBeam.get();
    }


    public void Intake() {

    }

    public void Score() {
        
    }
}
