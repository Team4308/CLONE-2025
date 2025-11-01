package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.EndEffector;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class OnlyIntake extends Command {
    private EndEffectorSubsystem m_EndEffectorSubsystem;
    private PivotSubsystem m_PivotSubsystem;

    public OnlyIntake(EndEffectorSubsystem m_EndEffectorSubsystem, PivotSubsystem m_PivotSubsystem) {
        this.m_EndEffectorSubsystem = m_EndEffectorSubsystem;
        this.m_PivotSubsystem = m_PivotSubsystem;
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().cancelAll();

        if (!m_EndEffectorSubsystem.getIntaken()) {
            intake().schedule();
        }
    }

    private Command intake() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> m_PivotSubsystem.setPivotTarget(Constants.Pivot.intakeAngle)),
                new InstantCommand(() -> m_EndEffectorSubsystem.Intake()),
                new WaitUntilCommand(() -> m_EndEffectorSubsystem.getIntaken()),
                new InstantCommand(() -> m_PivotSubsystem.setPivotTarget(Constants.Pivot.restAngle)),
                new InstantCommand(() -> m_EndEffectorSubsystem.StopMotors()),
                new WaitUntilCommand(() -> m_PivotSubsystem.atPosition()));
    }
}
