package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class OnlyScore extends SequentialCommandGroup {
    public OnlyScore(EndEffectorSubsystem m_EndEffectorSubsystem, PivotSubsystem m_PivotSubsystem) {
        addCommands(
                m_PivotSubsystem.movePivot(Constants.Pivot.scoreAngle),
                new WaitCommand(0.2),
                m_EndEffectorSubsystem.Score(),
                new WaitCommand(0.3),
                new InstantCommand(() -> m_EndEffectorSubsystem.setMotorSpeed(0)),
                m_PivotSubsystem.movePivot(Constants.Pivot.restAngle));
    }
}
