package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SystemsCheck extends SequentialCommandGroup {
        public SystemsCheck(EndEffectorSubsystem m_EndEffectorSubsystem, PivotSubsystem m_PivotSubsystem,
                        SwerveSubsystem swerveSubsystem, ClimbSubsystem m_ClimbSubsystem) {
                addCommands(
                                new InstantCommand(() -> Logger.recordOutput("SystemsCheck/Status", "Starting...")),

                                // Swerve Tests
                                new InstantCommand(() -> Logger.recordOutput("SystemsCheck/Status",
                                                "Testing Swerve Drive Forward...")),
                                new ParallelDeadlineGroup(new WaitCommand(1),
                                                swerveSubsystem.driveCommand(() -> 5, () -> 0, () -> 0)),

                                new InstantCommand(() -> Logger.recordOutput("SystemsCheck/Status",
                                                "Testing Swerve Drive Strafe...")),
                                new ParallelDeadlineGroup(new WaitCommand(1),
                                                swerveSubsystem.driveCommand(() -> 0, () -> 5, () -> 0)),

                                new InstantCommand(() -> Logger.recordOutput("SystemsCheck/Status",
                                                "Testing Swerve Drive Rotation...")),
                                new ParallelDeadlineGroup(new WaitCommand(1),
                                                swerveSubsystem.driveCommand(() -> 0, () -> 0, () -> 5)),

                                new WaitCommand(0.5),
                                new InstantCommand(() -> swerveSubsystem.lock()),

                                // Pivot and EndEffector
                                new InstantCommand(() -> Logger.recordOutput("SystemsCheck/Status",
                                                "Testing Intake Cycle...")),

                                new InstantCommand(() -> m_PivotSubsystem.setPivotTarget(Constants.Pivot.intakeAngle)),
                                new WaitUntilCommand(m_PivotSubsystem::atPosition),
                                new InstantCommand(m_EndEffectorSubsystem::Intake),
                                new WaitUntilCommand(m_EndEffectorSubsystem::getIntaken).withTimeout(10.0),
                                new InstantCommand(() -> {
                                        if (!m_EndEffectorSubsystem.getIntaken()) {
                                                Logger.recordOutput("SystemsCheck/Fault",
                                                                "Intake failed to detect Coral within 10 Seconds!");
                                        } else {
                                                Logger.recordOutput("SystemsCheck/Status", "Intake Succeeded.");
                                        }
                                }),

                                new InstantCommand(() -> m_PivotSubsystem.setPivotTarget(Constants.Pivot.restAngle)),
                                new WaitUntilCommand(m_PivotSubsystem::atPosition),
                                new InstantCommand(
                                                () -> Logger.recordOutput("SystemsCheck/Status", "Testing Release...")),
                                new InstantCommand(m_EndEffectorSubsystem::Score),
                                new WaitUntilCommand(() -> !m_EndEffectorSubsystem.getIntaken()).withTimeout(5.0),
                                new InstantCommand(() -> {
                                        if (m_EndEffectorSubsystem.getIntaken()) {
                                                Logger.recordOutput("SystemsCheck/Fault",
                                                                "Score failed to release Coral within 5 Seconds!");
                                        } else {
                                                Logger.recordOutput("SystemsCheck/Status", "Score Succeeded.");
                                        }
                                }),

                                new WaitCommand(1),
                                // Stop Motors
                                new InstantCommand(m_EndEffectorSubsystem::StopMotors),

                                // Climb Test
                                new InstantCommand(() -> Logger.recordOutput("SystemsCheck/Status",
                                                "Testing Climb Up...")),
                                new InstantCommand(m_ClimbSubsystem::climb),
                                new WaitCommand(1),
                                new InstantCommand(() -> Logger.recordOutput("SystemsCheck/Status",
                                                "Testing Climb Down...")),
                                new InstantCommand(m_ClimbSubsystem::release),
                                new WaitCommand(1),

                                // End
                                new InstantCommand(() -> {
                                        Logger.recordOutput("SystemsCheck/Status", "Finished.");
                                }));
        }
}
