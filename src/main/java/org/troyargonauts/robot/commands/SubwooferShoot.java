package org.troyargonauts.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.troyargonauts.robot.Robot;
import org.troyargonauts.robot.subsystems.*;

public class SubwooferShoot extends SequentialCommandGroup {
    public SubwooferShoot(){
        super(
                new InstantCommand(() -> Robot.getShooter().setState(Shooter.ShooterStates.SUBWOOFER), Robot.getShooter()),
                new InstantCommand(() -> Robot.getArm().setState(Arm.ArmStates.SUBWOOFER), Robot.getArm()).until(Robot.getArm()::isPIDFinished),
                // new WaitUntilCommand(Robot.getArm()::isPIDFinished),
                new ShootingSequence()
        );
    }
}
