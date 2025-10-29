// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.lang.management.OperatingSystemMXBean;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import ca.team4308.absolutelib.control.RazerWrapper;
import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Driver;
import frc.robot.commands.OnlyIntake;
import frc.robot.commands.Reset;
import frc.robot.commands.TogglePivot;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.Simulation;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
        // Controllers
        private final RazerWrapper driver = new RazerWrapper(Ports.Joysticks.DRIVER);

        // The robot's subsystems and commands are defined here...
        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve"));

        // Commands

        private final TogglePivot TogglePivotCommand;
        private final Reset ResetCommand;
        private final OnlyIntake OnlyIntakeCommand;

        private final SendableChooser<Command> autoChooser;

        private final Simulation m_simulation;
        private final PivotSubsystem m_pivotSubsystem;
        private final EndEffectorSubsystem m_endEffectorSubsystem;
        private final ClimbSubsystem m_ClimbSubsystem;

        private final Trigger drivebaseAlignedTrigger;
        private final Trigger isIntakenTrigger;
        private final Trigger last15SecondsTrigger;

        // Converts driver input into a field-relative ChassisSpeeds that is controlled
        // by angular velocity.
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driver.getLeftY() * -1,
                        () -> driver.getLeftX() * -1)
                        .withControllerRotationAxis(() -> driver.getRightX() * -1)
                        .deadband(Driver.DEADBAND)
                        .scaleTranslation(1.0)
                        .allianceRelativeControl(true);

        // Clone's the angular velocity input stream and converts it to a fieldRelative
        // input stream.
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driver::getRightX,
                        driver::getRightY)
                        .headingWhile(true);

        // Clone's the angular velocity input stream and converts it to a roboRelative
        // input stream.
        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                        .allianceRelativeControl(false);

        SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> -driver.getLeftY(),
                        () -> -driver.getLeftX())
                        .withControllerRotationAxis(() -> -driver.getRightX())
                        .deadband(Driver.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        // Derive the heading axis with math!
        SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                        .withControllerHeadingAxis(() -> Math.sin(
                                        driver.getLeftTrigger() * Math.PI) * (Math.PI * 2),
                                        () -> Math.cos(driver.getLeftTrigger() * Math.PI) * (Math.PI * 2))
                        .headingWhile(true);

        // Reef align
        SwerveInputStream driveToClosestLeftReef = driveDirectAngle.copy();
        SwerveInputStream driveToClosestRightReef = driveDirectAngle.copy();

        public RobotContainer() {
                m_simulation = new Simulation();
                m_pivotSubsystem = new PivotSubsystem();
                m_endEffectorSubsystem = new EndEffectorSubsystem();
                m_ClimbSubsystem = new ClimbSubsystem();

                TogglePivotCommand = new TogglePivot(m_endEffectorSubsystem, m_pivotSubsystem);
                ResetCommand = new Reset(m_endEffectorSubsystem, m_pivotSubsystem);
                OnlyIntakeCommand = new OnlyIntake(m_endEffectorSubsystem, m_pivotSubsystem);

                drivebaseAlignedTrigger = new Trigger(drivebase::isAligned);
                isIntakenTrigger = new Trigger(m_endEffectorSubsystem::getIntaken);
                last15SecondsTrigger = new Trigger(() -> Timer.getMatchTime() == 15);

                configureNamedCommands();
                configureDriverBindings();
                configureOtherTriggers();

                DriverStation.silenceJoystickConnectionWarning(true);
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);

                m_simulation.setupSubsystems(m_pivotSubsystem, m_endEffectorSubsystem);
        }

        private void configureDriverBindings() {
                /*
                 * 
                 * left joystick: strafe
                 * right joystick: rotation
                 * A (bottom): intake/score
                 * X, B: reef align
                 * Y: align to coral
                 */

                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

                Command driveFieldOrientedAnglularVelocityKeyboard = drivebase
                                .driveFieldOriented(driveAngularVelocityKeyboard);

                driver.M2.onTrue(TogglePivotCommand);

                if (Robot.isSimulation()) {
                        driver.M1.onTrue(new InstantCommand(() -> m_endEffectorSubsystem.simIntaking = true));
                        driver.M1.onFalse(new InstantCommand(() -> m_endEffectorSubsystem.simIntaking = false));
                }
                driver.M1.onTrue(ResetCommand);

                driver.RightTriggerTrigger.and(() -> m_pivotSubsystem.atPosition())
                                .whileTrue(Commands.run(() -> drivebase.driveTowardsTarget(
                                                () -> deadZone(driver.getRightTrigger())),
                                                drivebase));
                driver.RightTriggerTrigger.onTrue(OnlyIntakeCommand);

                driver.M3.whileTrue(drivebase.updateClosestReefPoses()
                                .andThen(drivebase.driveToPose(() -> drivebase.nearestPoseToLeftReef)));
                driver.M4.whileTrue(drivebase.updateClosestReefPoses()
                                .andThen(drivebase.driveToPose(() -> drivebase.nearestPoseToRightReef)));

                driver.LB.onTrue((Commands.runOnce(drivebase::zeroGyro)));
                driver.RB.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

                driver.M5.onTrue(new InstantCommand(m_ClimbSubsystem::release));
                driver.M6.onTrue(new InstantCommand(m_ClimbSubsystem::climb));

                if (RobotBase.isSimulation()) {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
                } else {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
                }
        }

        private void configureOtherTriggers() {
                drivebaseAlignedTrigger.and(isIntakenTrigger).onTrue(TogglePivotCommand);
                last15SecondsTrigger.onTrue(new InstantCommand(m_ClimbSubsystem::release));
        }

        public void configureTeleopBindings() {

        }

        public void configureNamedCommands() {
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void robotPeriodic() {

        }

        public void teleopInit() {
                configureTeleopBindings();
        }

        public void teleopPeriodic() {
        }

        public void simulationPerodic() {
        }

        public void disabledInit() {
                driver.setRumble(RumbleType.kBothRumble, 0);
        }

        public void runSystemsCheck() {

        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }

        private static double deadZone(double integer) {
                if (Math.abs(integer) < 0.1) {
                        integer = 0;
                }
                return integer;
        }
}