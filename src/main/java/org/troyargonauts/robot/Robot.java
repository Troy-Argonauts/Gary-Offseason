// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.troyargonauts.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import org.troyargonauts.robot.commands.ShootInPlaceAuton;
import org.troyargonauts.robot.commands.SubwooferShoot;
import org.troyargonauts.robot.commands.StartingSequence;
import org.troyargonauts.robot.commands.TuneDrive;
import org.troyargonauts.robot.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Class representing the entire robot - CommandBasedRobot framework
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private SendableChooser<Command> autoChooser;
    private final ScheduledExecutorService scheduledExecutorService = Executors.newSingleThreadScheduledExecutor();

    private static Arm arm;
    private static Climber climber;
    private static Intake intake;
    private static Shooter shooter;
    private static RobotContainer robotContainer;

    private boolean armLimitPressed;

    private DoubleLogEntry drivetrainFRDCurrentLog;
    private DoubleLogEntry drivetrainFRTCurrentLog;
    private DoubleLogEntry drivetrainBRDCurrentLog;
    private DoubleLogEntry drivetrainBRTCurrentLog;
    private DoubleLogEntry drivetrainFLDCurrentLog;
    private DoubleLogEntry drivetrainFLTCurrentLog;
    private DoubleLogEntry drivetrainBLDCurrentLog;
    private DoubleLogEntry drivetrainBLTCurrentLog;

    private DoubleLogEntry drivetrainFRDVoltageLog;
    private DoubleLogEntry drivetrainFRTVoltageLog;
    private DoubleLogEntry drivetrainBRDVoltageLog;
    private DoubleLogEntry drivetrainBRTVoltageLog;
    private DoubleLogEntry drivetrainFLDVoltageLog;
    private DoubleLogEntry drivetrainFLTVoltageLog;
    private DoubleLogEntry drivetrainBLDVoltageLog;
    private DoubleLogEntry drivetrainBLTVoltageLog;

//    UsbCamera camera1;
//    UsbCamera camera2;
//    VideoSink server;


    /**
     * This method is the first that is run when the robot is powered on. Only runs once.
     * Used to instantiate all subsystems, creates a thread for all PID control loops, sets up all available autonomous routines, and the autonomous chooser.
     */
    @Override
    public void robotInit() {
        DataLogManager.start();
        DataLog log = DataLogManager.getLog();


        arm = new Arm();
        climber = new Climber();
        intake = new Intake();
        shooter = new Shooter();
      
        robotContainer = new RobotContainer();

//        robotContainer.drivetrain.getPigeon2().setYaw(0);
//        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
//            robotContainer.drivetrain.getPigeon2().setYaw(0);
//        }

//        CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
//        configs.withStatorCurrentLimit(30);

//        camera1 = CameraServer.startAutomaticCapture(1);
//        camera2 = CameraServer.startAutomaticCapture(2);
//        server = CameraServer.getServer();


        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("ShootInPlace", new ShootInPlaceAuton());
        autoChooser.addOption("Nothing", new WaitCommand(15));

//        autoChooser.addOption("StartingSequence", new StartingSequence());
//        autoChooser.addOption("TuneDrive", new TuneDrive());
//        autoChooser.addOption("2 Note Arm 0 Auto", new PathPlannerAuto("2 Note Arm 0 Auto"));
//       autoChooser.addOption("Copy of 2 Note Arm f0 Auto", new PathPlannerAuto("Copy of 2 Note Arm 0 Auto"));
//        autoChooser.addOption("Test", new PathPlannerAuto("Test Auto - 6 feet"));
//        autoChooser.addOption("1Note Move Auto", new PathPlannerAuto("1Note Move Auto"));
//        autoChooser.addOption("2 Note Arm0 Auto", new PathPlannerAuto("2 Note ARM0 Auto"));
        autoChooser.addOption("B 2 Note W2 Center", new PathPlannerAuto("B 2 Note W2 Center"));
        autoChooser.addOption("B 3 Note W2 C1 Center", new PathPlannerAuto("B 3 Note W2 C1 Center"));
        autoChooser.addOption("Left Side 3 Note C4C5 Auto", new PathPlannerAuto("Left Side 3 Note C4C5 Auto"));
        autoChooser.addOption("B P2 W3 W2 W1 Auto", new PathPlannerAuto("B P2 W3 W2 W1 Auto"));
        autoChooser.addOption("Test Auto - 6 feet", new PathPlannerAuto("Test Auto - 6 feet"));

        SmartDashboard.putData("Auto Chooser", autoChooser);

        // CSA pbonnen 2024-03-16 for your reference
        /*scheduledExecutorService.scheduleAtFixedRate(new Runnable() {
            @Override
            public void run() {
                if (Robot.getArm().getLimitSwitch()) {
                    innerArmLimitPressed = true;
                    Robot.getArm().resetEncoders();
                }

                if (innerArmLimitPressed) {
                    arm.run();
                }

                shooter.run();
            }

            private boolean innerArmLimitPressed = false;
        }, 100, 10, TimeUnit.MILLISECONDS);*/
        new Trigger(Robot.getArm()::getLimitSwitch).onTrue(
            Commands.runOnce(()->Robot.getArm().resetEncoders())
        );
        scheduledExecutorService.scheduleAtFixedRate(() -> {
            if (Robot.getArm().getLimitSwitch()) {
                armLimitPressed = true;
            }

            if (armLimitPressed) {
               arm.run();
            }
            
//            if(robotContainer.getOperatorX()){
//                System.out.println("Xpressed");
//                shooter.run();
//            }
            shooter.run();


            //climber.run();
        }, 100, 10, TimeUnit.MILLISECONDS);

//        robotContainer.getDrivetrain().getModule(0).getSteerMotor().getConfigurator().apply(configs);
//        robotContainer.getDrivetrain().getModule(1).getSteerMotor().getConfigurator().apply(configs);
//        robotContainer.getDrivetrain().getModule(2).getSteerMotor().getConfigurator().apply(configs);
//        robotContainer.getDrivetrain().getModule(3).getSteerMotor().getConfigurator().apply(configs);

//        drivetrainFRDCurrentLog = new DoubleLogEntry((log), "Front Right Drive Motor Current");
//        drivetrainFRTCurrentLog = new DoubleLogEntry((log), "Front Right Turn Motor Current");
//        drivetrainBRDCurrentLog = new DoubleLogEntry((log), "Back Right Drive Motor Current");
//        drivetrainBRTCurrentLog = new DoubleLogEntry((log), "Front Right Turn Motor Current");
//        drivetrainFLDCurrentLog = new DoubleLogEntry((log), "Front Left Drive Motor Current");
//        drivetrainFLTCurrentLog = new DoubleLogEntry((log), "Front Left Turn Motor Current");
//        drivetrainBLDCurrentLog =  new DoubleLogEntry((log), "Back Left Drive Motor Current");
//        drivetrainBLTCurrentLog =  new DoubleLogEntry((log), "Back Left Turn Motor Current");

//        drivetrainFRDVoltageLog =  new DoubleLogEntry((log), "Front Right Drive Motor Voltage");
//        drivetrainFRTVoltageLog =  new DoubleLogEntry((log), "Front Right Turn Motor Voltage");
//        drivetrainBRDVoltageLog =  new DoubleLogEntry((log), "Back Right Drive Motor Voltage");
//        drivetrainBRTVoltageLog =  new DoubleLogEntry((log), "Back Right Turn Motor Voltage");
//        drivetrainFLDVoltageLog =  new DoubleLogEntry((log), "Front Left Drive Motor Voltage");
//        drivetrainFLTVoltageLog =  new DoubleLogEntry((log), "Front Left Turn Motor Voltage");
//        drivetrainBLDVoltageLog =  new DoubleLogEntry((log), "Back Left Drive Motor Voltage");
//        drivetrainBLTVoltageLog =  new DoubleLogEntry((log), "Back Left Turn Motor Voltage");



    }

    /**
     * This method runs periodically after the robot has been initialized at start-up. Runs the Command Scheduler.
     */
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("current", robotContainer.drivetrain.getModule(0).getDriveMotor().getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("gyro", robotContainer.drivetrain.getPigeon2().getAngle());
        SmartDashboard.putString("Alliance Side", DriverStation.getAlliance().get().toString());
        SmartDashboard.putNumber(" Arm Target", Robot.getArm().getCurrentTarget());

        if (DriverStation.getAlliance().isPresent()){
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                RobotContainer.inverted = -1;
                SmartDashboard.putNumber("Inverted", RobotContainer.inverted);
            } else {
                RobotContainer.inverted = 1;
                SmartDashboard.putNumber("Inverted", RobotContainer.inverted);

            } 
        }
    }

    /**
     * This method runs periodically when the robot is in Disabled mode.
     */
    @Override
    public void disabledPeriodic() {}

    /**
     * This method runs once when exiting Disabled mode.
     */
    @Override
    public void disabledInit() {
        //shooter.setDesiredTarget(0,0);
    }

    @Override
    public void disabledExit() {}

    /**
     * This method runs once at the start of Autonomous mode. Selects the autonomous routine to run based on the chooser
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = autoChooser.getSelected();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
     * This method runs periodically during Autonomous mode
     */
    @Override
    public void autonomousPeriodic() {

    }

    /**
     * This method runs once when exiting Autonomous mode.
     */
    @Override
    public void autonomousExit() {}

    /**
     * This method runs once at the start of Teleoperated mode. Cancels any Autonomous command that may be running.
     */
    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

//        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
////            robotContainer.drivetrain.getPigeon2().setYaw(robotContainer.drivetrain.getPigeon2().getAngle() + 180);
//        }
    }

    /**
     * This method runs once when exiting Teleoperated mode.
     */
    @Override
    public void teleopExit() {}

    /**
     * This method runs periodically during Teleoperated mode.
     */
    @Override
    public void teleopPeriodic() {
        if(!robotContainer.getOperatorRightBumper() && Robot.getIntake().isNoteReady() && !robotContainer.getOperatorRightTrigger()){
            Robot.getIntake().setState(Intake.IntakeStates.OFF);
            robotContainer.getOperator().getHID().setRumble(GenericHID.RumbleType.kBothRumble,0.5);
            robotContainer.getDriver().getHID().setRumble(GenericHID.RumbleType.kBothRumble,0.5);
        }
        else{
            robotContainer.getOperator().getHID().setRumble(GenericHID.RumbleType.kBothRumble,0);
            robotContainer.getDriver().getHID().setRumble(GenericHID.RumbleType.kBothRumble,0);
        }
//        if(robotContainer.getDriverDPadDown()) {
//            System.out.println("Setting Camera 2");
//            server.setSource(camera2);
//        } else if (robotContainer.getDriverDPadUp()) {
//            System.out.println("Setting Camera 1");
//            server.setSource(camera1);
//        }

    }

    /**
     * This method runs once at start of Test mode. Cancels all previously running commands.
     */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This method runs once when exiting Test mode.
     */
    @Override
    public void testExit() {}

    /**
     * This method runs periodically during a simulation.
     */
    @Override
    public void simulationPeriodic() {}

    /**
     * Gets the created Shooter subsystem. If a Shooter subsystem has not been created, creates one.
     *
     * @return Shooter Subsystem
     */
    public static Shooter getShooter() {
        if (shooter == null) shooter = new Shooter();
        return shooter;
    }
    /**
     * Gets the created Intake subsystem. If an Intake subsystem has not been created, creates one.
     *
     * @return Intake Subsystem
     */
    public static Intake getIntake() {
        if (intake == null) intake= new Intake();
        return intake;
    }

    /**
     * Gets the created Climber subsystem. If a Climber subsystem has not been created, creates one.
     *
     * @return Climber Subsystem
     */
    public static Climber getClimber() {
        if (climber == null) climber = new Climber();
        return climber;
    }

    /**
     * Gets the created Arm subsystem. If an Arm subsystem has not been created, creates one.
     *
     * @return Arm Subsystem
     */
    public static Arm getArm() {
        if (arm == null) arm = new Arm();
        return arm;
    }
    public static RobotContainer getRobotContainer(){
        return robotContainer;
    }
}
