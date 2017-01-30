package org.usfirst.frc.team2850.robot.commands;

import org.usfirst.frc.team2850.robot.Robot;
import org.usfirst.frc.team2850.robot.subsystems.ElevatorShooter;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Shoot extends Command {

	private double power;
    public Shoot(double power) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.elevatorshooter);
    	this.power = power;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevatorshooter.shoot(power);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	System.out.println("RateInner: " + Robot.elevatorshooter.shootInnerEncoder.getRate());
    	System.out.println("RateOuter: " + Robot.elevatorshooter.shootOuterEncoder.getRate());
    	System.out.println("OuterGain: " + Robot.elevatorshooter.outerGain);
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.elevatorshooter.shootStop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.elevatorshooter.shootStop();
    }
}
