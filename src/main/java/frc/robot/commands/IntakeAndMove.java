package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.EndEffector;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.OnlyIntake;

public class IntakeAndMove extends Command {
    private EndEffectorSubsystem m_EndEffectorSubsystem;
    private PivotSubsystem m_PivotSubsystem;
    private SwerveSubsystem m_SwerveSubsystem;

    public IntakeAndMove(EndEffectorSubsystem m_EndEffectorSubsystem, PivotSubsystem m_PivotSubsystem,
            SwerveSubsystem m_SwerveSubsystem) {
        this.m_EndEffectorSubsystem = m_EndEffectorSubsystem;
        this.m_PivotSubsystem = m_PivotSubsystem;
        this.m_SwerveSubsystem = m_SwerveSubsystem;
    }

    @Override
    public void initialize() {
        OnlyIntake x = new OnlyIntake(m_EndEffectorSubsystem, m_PivotSubsystem);
        x.schedule();
    }

    @Override
    public void execute() {
        if (m_PivotSubsystem.atPosition()) {
            m_SwerveSubsystem.driveTowardsTarget(() -> 0.67);
        }
    }

    @Override
    public void end(boolean inter) {
        if (m_EndEffectorSubsystem.getIntaken()) {
            // m_PivotSubsystem.movePivot(Constants.Pivot.restAngle).schedule();
            // m_EndEffectorSubsystem.StopMotors();
        }

    }

    @Override
    public boolean isFinished() {
        return m_EndEffectorSubsystem.getIntaken();
    }
}
