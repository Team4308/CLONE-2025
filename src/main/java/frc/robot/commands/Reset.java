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

public class Reset extends Command {
    private EndEffectorSubsystem m_EndEffectorSubsystem;
    private PivotSubsystem m_PivotSubsystem;
    private ClimbSubsystem m_ClimbSubsystem;

    public Reset(EndEffectorSubsystem m_EndEffectorSubsystem, PivotSubsystem m_PivotSubsystem,
            ClimbSubsystem m_ClimbSubsystem) {
        this.m_EndEffectorSubsystem = m_EndEffectorSubsystem;
        this.m_PivotSubsystem = m_PivotSubsystem;
        this.m_ClimbSubsystem = m_ClimbSubsystem;
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().cancelAll();

        res().schedule();
    }

    private Command res() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> m_PivotSubsystem.setPivotTarget(Constants.Pivot.restAngle)),
                new InstantCommand(() -> m_EndEffectorSubsystem.setMotorSpeed(0)),
                new InstantCommand(() -> m_ClimbSubsystem.stop()));
    }
}
