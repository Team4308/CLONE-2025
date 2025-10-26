// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import ca.team4308.absolutelib.control.XBoxWrapper;

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

import frc.robot.subsystems.Simulation;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
        // Controllers
        private final XBoxWrapper driver = new XBoxWrapper(Ports.Joysticks.DRIVER);
        private final XBoxWrapper operator = new XBoxWrapper(Ports.Joysticks.OPERATOR);

        // The robot's subsystems and commands are defined here...
        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve"));
       
        // Commands

        private final SendableChooser<Command> autoChooser;

        private final Simulation m_simulation;

        private final Trigger drivebaseAlignedTrigger;

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
                        .withControllerRotationAxis(() -> driver.getRightX())
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

                drivebaseAlignedTrigger = new Trigger(drivebase::isAligned);

                configureNamedCommands();
                configureDriverBindings();
                configureOperatorBindings();
                configureOtherTriggers();

                DriverStation.silenceJoystickConnectionWarning(true);
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        private void configureDriverBindings() {
                // Command driveFieldOrientedDirectAngle =
                // drivebase.driveFieldOriented(driveDirectAngle);
                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
                // Command driveRobotOrientedAngularVelocity =
                // drivebase.driveFieldOriented(driveRobotOriented);
                // Command driveSetpointGen =
                // drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
                // Command driveFieldOrientedDirectAngleKeyboard =
                // drivebase.driveFieldOriented(driveDirectAngleKeyboard);
                Command driveFieldOrientedAnglularVelocityKeyboard = drivebase
                                .driveFieldOriented(driveAngularVelocityKeyboard);
                // Command driveSetpointGenKeyboard = drivebase
                // .driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

                // driver.A.whileTrue(drivebase.updateClosestAlgaePose()
                // .andThen(drivebase.driveToPose(() -> drivebase.nearestPoseToAlgaeRemove)));
                driver.A.whileTrue(drivebase.updateClosestReefPoses()
                                .andThen(drivebase.driveToPose(() -> drivebase.nearestPoseToLeftReef)));
                driver.B.whileTrue(drivebase.updateClosestReefPoses()
                                .andThen(drivebase.driveToPose(() -> drivebase.nearestPoseToRightReef)));
                driver.X.whileTrue(drivebase.updateClosestStationPose()
                                .andThen(drivebase.driveToPose(() -> drivebase.nearestPoseToFarCoralStation)));
                driver.Y.whileTrue(drivebase.updateClosestStationPose()
                                .andThen(drivebase.driveToPose(() -> drivebase.nearestPoseToNearCoralStation)));
                driver.LB.onTrue((Commands.runOnce(drivebase::zeroGyro)));
                driver.RB.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

                if (RobotBase.isSimulation()) {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
                } else {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
                }
        }

        private void configureOperatorBindings() {
        }

        private void configureOtherTriggers() {
 
                drivebaseAlignedTrigger.onTrue(new InstantCommand(() -> System.out.println("Drivebase Aligned")));
                drivebaseAlignedTrigger.onFalse(new InstantCommand(() -> {
   

                }));
        }

        public void configureTeleopBindings() {
          
                drivebaseAlignedTrigger.onTrue(new InstantCommand(() -> operator.setRumble(RumbleType.kBothRumble, 1)));
                drivebaseAlignedTrigger
                                .onFalse(new InstantCommand(() -> operator.setRumble(RumbleType.kBothRumble, 0)));

                // drivebaseAlignedTrigger.onTrue(new InstantCommand(() ->
                // driver.setRumble(RumbleType.kBothRumble, 1)));
                // drivebaseAlignedTrigger.onFalse(new InstantCommand(() ->
                // driver.setRumble(RumbleType.kBothRumble, 0)));

                operator.X.onTrue(new InstantCommand(() -> operator.setRumble(RumbleType.kBothRumble, 0)));
                operator.A.onTrue(new InstantCommand(() -> operator.setRumble(RumbleType.kBothRumble, 0)));
                operator.Y.onTrue(new InstantCommand(() -> operator.setRumble(RumbleType.kBothRumble, 0)));
                operator.RB.onTrue(new InstantCommand(() -> operator.setRumble(RumbleType.kBothRumble, 0)));
                operator.LB.onTrue(new InstantCommand(() -> operator.setRumble(RumbleType.kBothRumble, 0)));
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
                operator.setRumble(RumbleType.kBothRumble, 0);
        }

        private double joystickAlgaeArm() {
                return -deadZone(operator.getLeftY()) * 7;
        }

        public double joystickElevatorControl() {
                return -deadZone(operator.getRightY()) / 10;
        }

        private double triggerRollerControl() {
                double isPos = deadZone(operator.getRightTrigger()) * 50;
                double isNeg = deadZone(operator.getLeftTrigger()) * 50;
                if (isPos > 0) {
                        return isPos;
                } else {
                        return -isNeg;
                }
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