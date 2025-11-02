package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public class TogglePivot extends Command {
    private EndEffectorSubsystem m_EndEffectorSubsystem;
    private PivotSubsystem m_PivotSubsystem;

    public TogglePivot(EndEffectorSubsystem m_EndEffectorSubsystem, PivotSubsystem m_PivotSubsystem) {
        this.m_EndEffectorSubsystem = m_EndEffectorSubsystem;
        this.m_PivotSubsystem = m_PivotSubsystem;
    }

    @Override
    public void initialize() {
        if (!m_EndEffectorSubsystem.getIntaken()) {
            OnlyIntake intakeCommand = new OnlyIntake(m_EndEffectorSubsystem, m_PivotSubsystem);
            intakeCommand.schedule();
        } else {
            OnlyScore scoreCommand = new OnlyScore(m_EndEffectorSubsystem, m_PivotSubsystem);
            scoreCommand.schedule();
        }
    }

    public boolean isFinished() {
        return true;
    }
}
