package org.troyargonauts.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.troyargonauts.robot.Constants;

import static org.troyargonauts.robot.Constants.Arm.*;

/**
 * Class representing the Arm subsystem
 *
 * @author Ashwin Shrivastav
 */
public class Arm extends SubsystemBase {
    private TalonFX leftArmMotor, rightArmMotor;

    private DigitalInput limitSwitch;

    private double leftArmEncoder, rightArmEncoder = 0;
    private double armTarget = 0;
    private double oldTarget;

    private DoubleLogEntry armLeftEncoderLog;
    private DoubleLogEntry armRightEncoderLog;
    private DoubleLogEntry armLeftOutputCurrentLog;
    private DoubleLogEntry armRightOutputCurrentLog;
    private DoubleLogEntry armLeftMotorVoltage;
    private DoubleLogEntry armRightMotorVoltage;
//    public Double armLimitPressed = new Double(0);

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);
    TalonFXConfiguration config = new TalonFXConfiguration();
    MotionMagicConfigs mmConfig = config.MotionMagic;


    /**
     * Instantiates and configures motor controllers and sensors; creates Data Logs. Assigns PID constants.
     */
    public Arm() {
        mmConfig.MotionMagicCruiseVelocity = 60; // 5 (mechanism) rotations per second cruise
        mmConfig.MotionMagicAcceleration = 140; // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.1 seconds to reach max accel 
        mmConfig.MotionMagicJerk = 300;

        Slot0Configs slot0 = config.Slot0;
        slot0.kS = 0.; // Add 0.25 V output to overcome static friction
        slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 14; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0; // No output for integrated error
        slot0.kD = 0.2; // A velocity error of 1 rps results in 0.5 V output

//        allConfigs.Voltage.PeakForwardVoltage = 4;
//        allConfigs.MotorOutput.PeakForwardDutyCycle = 0.3;
//        allConfigs.Voltage.PeakReverseVoltage = -4;
//       allConfigs.MotorOutput.PeakForwardDutyCycle = 0.3;
//       allConfigs.MotorOutput.PeakReverseDutyCycle = -0.3;




//        positionVoltage.withLimitForwardMotion(true);


        leftArmMotor = new TalonFX(LEFT_MOTOR_ID, CANBUS_NAME);
        rightArmMotor = new TalonFX(RIGHT_MOTOR_ID, CANBUS_NAME);

        leftArmMotor.getConfigurator().apply(config);
        rightArmMotor.getConfigurator().apply(config);

        rightArmMotor.getClosedLoopError().setUpdateFrequency(100);

        //Left following right
        leftArmMotor.setControl(new Follower(rightArmMotor.getDeviceID(), true));

        rightArmMotor.setInverted(true);


        leftArmMotor.setNeutralMode(NeutralModeValue.Brake);
        rightArmMotor.setNeutralMode(NeutralModeValue.Brake);

        limitSwitch = new DigitalInput(LIMIT_SWITCH_SLOT);

        DataLog log = DataLogManager.getLog();

        armLeftOutputCurrentLog = new DoubleLogEntry(log, "Arm Left Motor Output Current ");
        armRightOutputCurrentLog = new DoubleLogEntry(log, "Arm Right Motor Output Current ");
        armLeftMotorVoltage = new DoubleLogEntry(log, "Arm Left Motor Bus Voltage");
        armRightMotorVoltage = new DoubleLogEntry(log, "Arm Right Motor Bus Voltage");

    }

    /**
     * Updates the encoders and outputs their positions to the SmartDashboard periodically. Append values to each data log periodically
     */
    @Override
    public void periodic() {
        leftArmEncoder = leftArmMotor.getPosition().getValueAsDouble();
        rightArmEncoder = rightArmMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("oldtarget",oldTarget);
        SmartDashboard.putNumber("current Target", armTarget);
        SmartDashboard.putNumber("Average Arm Encoder: ", (leftArmEncoder+rightArmEncoder)/2);
        SmartDashboard.putBoolean("Arm Limit: ", getLimitSwitch());
        SmartDashboard.putNumber("Voltage Cap", leftArmMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putBoolean("ARMPIDState", isPIDFinished());
        SmartDashboard.putNumber("current", rightArmMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("ARM sPEED", rightArmMotor.getDutyCycle().getValueAsDouble());
        

        armLeftOutputCurrentLog.append(leftArmMotor.getStatorCurrent().getValue());
        armRightOutputCurrentLog.append(rightArmMotor.getStatorCurrent().getValue());
        armLeftMotorVoltage.append(leftArmMotor.getMotorVoltage().getValue());
        armRightMotorVoltage.append(rightArmMotor.getMotorVoltage().getValue());

    }

    /**
     * Resets the encoders of both the left and right Arm motors.
     * Sets both motor positions to 0.
     */
    public void resetEncoders() {
        leftArmMotor.setPosition(0);
        rightArmMotor.setPosition(0);
    }

    /**
     * Sets raw motor power for both left and right Arm motors
     *
     * @param power desired power (value between -1 and 1)
     */
    public void setPower(double power) {
        rightArmMotor.set(power);
    }

    /**
     * Sets the PID loops for the left and right Arm motors to their corresponding target positions
     */
    public void run() {

        if (armTarget == 0 && leftArmEncoder <= 1 && rightArmEncoder <= 1){
            rightArmMotor.setControl(new StaticBrake());
        } else {
            rightArmMotor.setControl(mmRequest.withPosition(armTarget));
        }
        
        //leftArmMotor.setControl(mmRequest.withPosition(armTarget));
        //rightArmMotor.setControl(mmRequest.withPosition(armTarget));
    }

    /**
     * Checks if the PID loops for Left and Right Arm motors are within the window for their setpoints
     *
     * @return Whether the PIDs are finished
     */
    public boolean isPIDFinished() {
        return (Math.abs(armTarget - rightArmMotor.getPosition().getValueAsDouble()) <= 0.5);

    }
    
    /**
     * Changes the setpoint based on joystick value
     * .
     * @param joystickValue joystick value between -1 and 1
     */
    public void adjustSetpoint(double joystickValue) {
        if (!limitSwitch.get() || joystickValue != 0) {
            armTarget += (joystickValue * 0.2);
        }
    }

    /**
     * Sets enumerators for encoder positions of various Arm States
     */
    public enum ArmStates{
        /**
         * Floor Intake Arm position
         */
        FLOOR_INTAKE(0),

        /**
         * Amp scoring Arm position
         */
        AMP(23.11), //24.4

        /**
         * Stage scoring Arm position
         */
        STAGE(8.3), //want 7.4

        /**
         * Subwoofer scoring Arm position
         */
        SUBWOOFER(0),

        /**
         * Wing scoring Arm position
         */
        WING_LINE(9.8),

        /**
         * Wing scoring Arm position
         */
        WING_NOTE1(7.9),// possibly 6.5
         WING_NOTE2(7.5),
         WING_NOTE3(7.9),

        /**
         * Climbing Arm position
         */
        CLIMBER(0),

        START(0),

        /**
         * Feeder Arm position
         */
        FEEDER(0);

        final double armPosition;

        ArmStates(double armPosition){
            this.armPosition = armPosition;
        }
    }

    /**
     * Returns the average of the Arm encoder values.
     *
     * @return Average arm encoder value.
     */
    public double getEncoderValue() {
        return (rightArmEncoder + leftArmEncoder) / 2;
    }

    /**
     * Returns value of current Arm target position
     *
     * @return current Arm target
     */
    public double getCurrentTarget() {
        return armTarget;
    }

    /**
     * Sets target of arm to desired target that is not included in ArmStates
     *
     * @param desiredTarget Desired arm target
     */
    public void setDesiredTarget(double desiredTarget) {
        armTarget = desiredTarget;
    }

    /**
     * Sets the arm target to the desired Arm State position
     * @param state Desired Arm state
     */
    public void setState(ArmStates state) {
        oldTarget = armTarget;
        armTarget = state.armPosition;
    }

    public boolean checkWNState(){
        if(leftArmEncoder >= 7.0){
            return true;
        }
        else{
            return false;
        }
    }

    /**
     * Gets limit switch state
     *
     * @return Limit switch state
     */
    public boolean getLimitSwitch() {

        return !limitSwitch.get();
    }
}
