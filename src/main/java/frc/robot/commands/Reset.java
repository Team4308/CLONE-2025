package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.EndEffector;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class Reset extends SequentialCommandGroup {
    public Reset(EndEffectorSubsystem m_EndEffectorSubsystem, PivotSubsystem m_PivotSubsystem,
            ClimbSubsystem m_ClimbSubsystem) {
        CommandScheduler.getInstance().cancelAll();
        addCommands(
                new ParallelCommandGroup(m_PivotSubsystem.movePivot(Constants.Pivot.restAngle),
                        new InstantCommand(
                                () -> m_EndEffectorSubsystem.setMotorSpeed(Constants.EndEffector.ScoreSpeed)),
                        new InstantCommand(() -> m_ClimbSubsystem.stop())),
                new InstantCommand(() -> m_EndEffectorSubsystem.setMotorSpeed(Constants.EndEffector.ScoreSpeed)),
                new WaitCommand(0.5),
                new InstantCommand(() -> m_EndEffectorSubsystem.StopMotors()));
        CommandScheduler.getInstance().cancelAll();
        ;
    }
}
