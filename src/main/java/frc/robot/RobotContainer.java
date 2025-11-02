// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import ca.team4308.absolutelib.control.RazerWrapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Driver;
import frc.robot.FieldLayout.HP_DEPENDENT;
import frc.robot.FieldLayout.REEF_DEPENDENT;
import frc.robot.commands.IntakeAndMove;
import frc.robot.commands.OnlyScore;
import frc.robot.commands.Reset;
import frc.robot.commands.SystemsCheck;
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
        private final OnlyScore OnlyScoreCommand;
        private final IntakeAndMove IntakeAndMoveCommand;

        private SendableChooser<Command> place1;
        private SendableChooser<Command> pickup1;
        private SendableChooser<Command> place2;
        private SendableChooser<Command> pickup2;
        private SendableChooser<Command> place3;
        private SendableChooser<Command> pickup3;
        private SendableChooser<Command> place4;

        private final SendableChooser<Command> autoChooser;

        private final Simulation m_simulation;
        private final PivotSubsystem m_pivotSubsystem;
        private final EndEffectorSubsystem m_endEffectorSubsystem;
        private final ClimbSubsystem m_ClimbSubsystem;

        private final Trigger drivebaseAlignedTrigger;
        private final Trigger last15SecondsTrigger;

        // Converts driver input into a field-relative ChassisSpeeds that is controlled
        // by angular velocity.
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driver.getLeftY() * -1,
                        () -> driver.getLeftX() * -1)
                        .withControllerRotationAxis(() -> driver.getRightX())
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
                ResetCommand = new Reset(m_endEffectorSubsystem, m_pivotSubsystem, m_ClimbSubsystem);
                OnlyScoreCommand = new OnlyScore(m_endEffectorSubsystem, m_pivotSubsystem);
                IntakeAndMoveCommand = new IntakeAndMove(m_endEffectorSubsystem, m_pivotSubsystem, drivebase);

                drivebaseAlignedTrigger = new Trigger(drivebase::isAligned);
                last15SecondsTrigger = new Trigger(() -> DriverStation.getMatchTime() <= 100);

                //configureAutons();
                configureDriverBindings();
                configureOtherTriggers();

                DriverStation.silenceJoystickConnectionWarning(true);

                SmartDashboard.putData("Place 1", place1);
                SmartDashboard.putData("Pickup 1", pickup1);
                SmartDashboard.putData("Place 2", place2);
                SmartDashboard.putData("Pickip 2", pickup2);
                SmartDashboard.putData("Place 3", place3);
                SmartDashboard.putData("Pickup 3", pickup3);
                SmartDashboard.putData("Place 4", place4);

                m_simulation.setupSubsystems(m_pivotSubsystem, m_endEffectorSubsystem);
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
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

                driver.M1.onTrue(ResetCommand);

                driver.M2.onTrue(TogglePivotCommand);

                if (Robot.isSimulation()) {
                        driver.X.onTrue(new InstantCommand(() -> m_endEffectorSubsystem.simIntaking = true));
                        driver.X.onFalse(new InstantCommand(() -> m_endEffectorSubsystem.simIntaking = false));
                }

                driver.RightTriggerTrigger.whileTrue(IntakeAndMoveCommand);

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
                drivebaseAlignedTrigger.onTrue(OnlyScoreCommand);
                last15SecondsTrigger.onTrue(new InstantCommand(m_ClimbSubsystem::release));
                // coralSpottedTrigger.onTrue(OnlyIntakeCommand);
        }

        public void configureTeleopBindings() {

        }

        public void configureAutons() {
                place1 = new SendableChooser<Command>();
                pickup1 = new SendableChooser<Command>();
                place2 = new SendableChooser<Command>();
                pickup2 = new SendableChooser<Command>();
                place3 = new SendableChooser<Command>();
                pickup3 = new SendableChooser<Command>();
                place4 = new SendableChooser<Command>();

                // PLACE CHOOSERS: all REEF_DEPENDENT poses
                List<SendableChooser<Command>> placeChoosers = Arrays.asList(place1, place2, place3, place4);
                Pose2d[] reefPoses = {
                                REEF_DEPENDENT.A(),
                                REEF_DEPENDENT.B(),
                                REEF_DEPENDENT.C(),
                                REEF_DEPENDENT.D(),
                                REEF_DEPENDENT.E(),
                                REEF_DEPENDENT.F(),
                                REEF_DEPENDENT.G(),
                                REEF_DEPENDENT.H(),
                                REEF_DEPENDENT.I(),
                                REEF_DEPENDENT.J(),
                                REEF_DEPENDENT.K(),
                                REEF_DEPENDENT.L()
                };
                String[] poseNames = { "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L" };

                for (SendableChooser<Command> chooser : placeChoosers) {
                        for (int i = 0; i < reefPoses.length; i++) {
                                if (i == 0) {
                                        chooser.setDefaultOption("Place " + poseNames[i],
                                                        drivebase.driveToPose(reefPoses[i]));
                                } else {
                                        chooser.addOption("Place " + poseNames[i], drivebase.driveToPose(reefPoses[i]));
                                }
                        }
                }

                // PICKUP CHOOSERS: all HP_DEPENDENT poses
                List<SendableChooser<Command>> pickupChoosers = Arrays.asList(pickup1, pickup2, pickup3);
                Pose2d[] hpPoses = {
                                HP_DEPENDENT.HP_FROM_LEFT(),
                                HP_DEPENDENT.HP_FROM_RIGHT(),
                                HP_DEPENDENT.HP_LEFT_FROM_MID(),
                                HP_DEPENDENT.HP_RIGHT_FROM_MID()
                };
                String[] hpNames = { "From Left", "From Right", "Left From Mid", "Right From Mid" };

                for (SendableChooser<Command> chooser : pickupChoosers) {
                        for (int i = 0; i < hpPoses.length; i++) {
                                if (i == 0) {
                                        chooser.setDefaultOption("HP " + hpNames[i], drivebase.driveToPose(hpPoses[i]));
                                } else {
                                        chooser.addOption("HP " + hpNames[i], drivebase.driveToPose(hpPoses[i]));
                                }
                        }
                }

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                /* 
                return new SequentialCommandGroup(place1.getSelected().until(() -> drivebase.isAligned()),
                                pickup1.getSelected(),
                                new IntakeAndMove(m_endEffectorSubsystem, m_pivotSubsystem, drivebase)
                                                .until(() -> m_endEffectorSubsystem.getIntaken()),
                                place2.getSelected().until(() -> drivebase.isAligned()),
                                pickup2.getSelected(),
                                new IntakeAndMove(m_endEffectorSubsystem, m_pivotSubsystem, drivebase)
                                                .until(() -> m_endEffectorSubsystem.getIntaken()),
                                place3.getSelected().until(() -> drivebase.isAligned()),
                                pickup3.getSelected(),
                                new IntakeAndMove(m_endEffectorSubsystem, m_pivotSubsystem, drivebase)
                                                .until(() -> m_endEffectorSubsystem.getIntaken()),
                                place4.getSelected().until(() -> drivebase.isAligned()));

                */
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

                new SystemsCheck(m_endEffectorSubsystem, m_pivotSubsystem, drivebase, m_ClimbSubsystem).schedule();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }
}