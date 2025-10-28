package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class ClimbSubsystem extends SubsystemBase {

    private DigitalInput topLimit = new DigitalInput(Ports.Climb.topBeamBreak);
    private DigitalInput botLimit = new DigitalInput(Ports.Climb.botBeamBreak);

    private double motorSpeed = 0;

    private final SparkMax armMotor = new SparkMax(Ports.Climb.Motor, MotorType.kBrushless);

    public ClimbSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);

        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        armMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    // Idk change this to have logic later
    public void climb() {
        motorSpeed = Ports.Climb.speed;
    }

    public void release() {
        motorSpeed = -Ports.Climb.speed;
    }

    @Override
    public void periodic() {
        if (topLimit.get()) {
            motorSpeed = Math.min(motorSpeed, 0);
        } else if (botLimit.get()) {
            motorSpeed = Math.max(motorSpeed, 0);
        }

        armMotor.set(motorSpeed);
    }
}
