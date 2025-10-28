package frc.robot.subsystems;

import java.lang.annotation.Target;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Simulation extends SubsystemBase {
    private EndEffectorSubsystem m_EndEffectorSubsystem;
    private PivotSubsystem m_PivotSubsystem;

    private LoggedMechanism2d mech;

    private SingleJointedArmSim m_pivotSim;
    private LoggedMechanismRoot2d m_pivotRoot;
    private LoggedMechanismLigament2d m_pivotMech2d;

    private Pose3d zeroPose = new Pose3d();
    private Pose3d pivotPose = new Pose3d();

    public Simulation() {
        mech = new LoggedMechanism2d(10, 10);

        m_pivotSim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                49.5,
                SingleJointedArmSim.estimateMOI(Units.inchesToMeters(20), Units.lbsToKilograms(13.468)),
                Units.inchesToMeters(10),
                Units.degreesToRadians(5),
                Units.degreesToRadians(140),
                true,
                Units.degreesToRadians(60),
                0.0,
                0.0);

        m_pivotRoot = mech.getRoot("Pivot Root", 5, 0);
        m_pivotMech2d = m_pivotRoot
                .append(new LoggedMechanismLigament2d("Arm", 10,
                        Units.radiansToDegrees(m_pivotSim.getAngleRads())));
    }

    public void setupSubsystems(PivotSubsystem m_PivotSubsystem, EndEffectorSubsystem m_EndEffectorSubsystem) {
        this.m_PivotSubsystem = m_PivotSubsystem;
        this.m_EndEffectorSubsystem = m_EndEffectorSubsystem;
    }

    @Override
    public void periodic() {
        if (Robot.isSimulation()) {
            m_pivotSim.setInputVoltage(m_PivotSubsystem.getPivotVoltage());

            m_pivotSim.update(0.020);

            pivotPose = new Pose3d(
                    new Translation3d(0.289, 0.295, 0.19),
                    new Rotation3d(m_pivotSim.getAngleRads() + Units.degreesToRadians(-50), 0,
                            Units.degreesToRadians(270)));
            m_pivotMech2d.setAngle(Units.radiansToDegrees(m_pivotSim.getAngleRads()));

            m_PivotSubsystem.simEncoderVal = (Units.radiansToDegrees(m_pivotSim.getAngleRads())) / 90;
        }

        Logger.recordOutput("Simulation/pivotPose", pivotPose);
        Logger.recordOutput("Simulation/Mechanism", mech);
        Logger.recordOutput("Simulation/Zeroed Pose", zeroPose);

    }
}
