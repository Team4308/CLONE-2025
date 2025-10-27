package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class EndeffectorSubsystem extends SubsystemBase {

    private TalonFX IntakeMotor = new TalonFX(Ports.EndEffector.IntakeMotor);
    private TalonFX CenteringMotor = new TalonFX(Ports.EndEffector.CenteringMotor);
    private DigitalInput leftBeam = new DigitalInput(Ports.EndEffector.leftBeam);
    private DigitalInput rightBeam = new DigitalInput(Ports.EndEffector.rightBeam);
    public boolean isIntaking = false;

    public EndeffectorSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = Constants.EndEffector.kP;
        config.Slot0.kI = Constants.EndEffector.kI;
        config.Slot0.kD = Constants.EndEffector.kD;
        
        IntakeMotor.getConfigurator().apply(config);
        CenteringMotor.getConfigurator().apply(config);


    }

    boolean getIntaken() {
        return leftBeam.get() || rightBeam.get();
    }


    public void Intake() {
        IntakeMotor.set(Constants.EndEffector.IntakeSpeed);
        isIntaking = true;
    }

    public void Score() {
        IntakeMotor.set(-Constants.EndEffector.ScoreSpeed);
    }


    @Override
    public void periodic() {
        // Stop intaking once the game piece is detected
        if (isIntaking && getIntaken()) {
            IntakeMotor.set(0);
            isIntaking = false;
        }
        
    }

}
