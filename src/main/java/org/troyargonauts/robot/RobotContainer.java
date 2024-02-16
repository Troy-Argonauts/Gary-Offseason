// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.troyargonauts.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.troyargonauts.common.math.OMath;
import org.troyargonauts.common.streams.IStream;
import org.troyargonauts.robot.generated.TunerConstants;
import org.troyargonauts.robot.subsystems.Climber;
import org.troyargonauts.robot.subsystems.Intake;

import static org.troyargonauts.robot.Robot.*;
import org.troyargonauts.robot.subsystems.Arm;
import org.troyargonauts.robot.subsystems.Intake;


public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController operator = new CommandXboxController(Constants.Controllers.DRIVER); // My joystick
    private final CommandXboxController driver = new CommandXboxController(Constants.Controllers.OPERATOR);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.Velocity); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
      drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
              drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                      // negative Y (forward)
                      .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                      .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
              ));

      driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
      driver.b().whileTrue(drivetrain
              .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

      // reset the field-centric heading on left bumper press
      driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
              drivetrain.applyRequest(() -> drive.withVelocityX(-operator.getLeftY() * MaxSpeed) // Drive forward with
                      // negative Y (forward)
                      .withVelocityY(-operator.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                      .withRotationalRate(-operator.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
              );

      operator.a().whileTrue(drivetrain.applyRequest(() -> brake));
      operator.b().whileTrue(drivetrain
              .applyRequest(() -> point.withModuleDirection(new Rotation2d(-operator.getLeftY(), -operator.getLeftX()))));

      // reset the field-centric heading on left bumper press
      operator.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

      if (Utils.isSimulation()) {
          drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
          drivetrain.registerTelemetry(logger::telemeterize);
      }

      driver.rightBumper().onTrue(
              drivetrain.applyRequest(() -> brake)
      );



      if(driver.getRightTriggerAxis() > 0){
          new RunCommand(
                  () -> {
                      double climberInput = IStream.create(driver::getRightTriggerAxis)
                              .filtered(x -> OMath.deadband(x, Constants.Climber.DEADBAND))
                              .get();
                      getClimber().setState(Climber.MotorStates.TOP);
                  }, Robot.getClimber()
          );
      }
      else if(driver.getLeftTriggerAxis() > 0){
          new RunCommand(
                  () -> {
                      double climberInput = IStream.create(driver::getLeftTriggerAxis)
                              .filtered(x -> OMath.deadband(x, Constants.Climber.DEADBAND))
                              .get();
                      getClimber().setState(Climber.MotorStates.BOTTOM);
                  }, Robot.getClimber()
          );
      }



      if(operator.getRightTriggerAxis() > 0){
              new RunCommand(() -> {
                  double intakeInput = IStream.create(operator::getRightTriggerAxis)
                          .filtered(x -> OMath.deadband(x, Constants.Intake.DEADBAND))
                          .get();
                  getIntake().setState(Intake.MotorState.IN);
              }, Robot.getIntake());
              }

      operator.a().onTrue(
              new ParallelCommandGroup(
                      new InstantCommand(() -> Robot.getArm().setState(Arm.ArmStates.FLOOR_INTAKE)),
                      new InstantCommand(() -> Robot.getIntake().setState(Intake.MotorState.IN)).until(() -> Robot.getIntake().isNoteReady())
              )
      );

  }





  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
