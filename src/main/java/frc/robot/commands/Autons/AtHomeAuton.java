package frc.robot.commands.Autons;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldLayout;
import frc.robot.commands.IntakeAndMove;
import frc.robot.commands.OnlyScore;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AtHomeAuton extends SequentialCommandGroup {

    public AtHomeAuton(PivotSubsystem pivotSubsystem, EndEffectorSubsystem endEffectorSubsystem,
            SwerveSubsystem m_SwerveSubsystem) {
        addCommands(
                m_SwerveSubsystem.driveToPose(() -> new Pose2d(2.3, 3.7, new Rotation2d(Math.toRadians(-82.73))))
                        .until(() -> m_SwerveSubsystem.isAligned()),
                new IntakeAndMove(endEffectorSubsystem, pivotSubsystem, m_SwerveSubsystem)
                        .until(() -> endEffectorSubsystem.getIntaken()),
                m_SwerveSubsystem.driveToPose(() -> FieldLayout.REEF_DEPENDENT.B())
                        .until(() -> !endEffectorSubsystem.getIntaken()),
                m_SwerveSubsystem.driveToPose(() -> new Pose2d(2.7, 3.45, new Rotation2d(Math.toRadians(133))))
                        .until(() -> m_SwerveSubsystem.isAligned()),
                new IntakeAndMove(endEffectorSubsystem, pivotSubsystem, m_SwerveSubsystem)
                        .until(() -> endEffectorSubsystem.getIntaken()),
                m_SwerveSubsystem.driveToPose(() -> FieldLayout.REEF_DEPENDENT.B()));
    }
}
