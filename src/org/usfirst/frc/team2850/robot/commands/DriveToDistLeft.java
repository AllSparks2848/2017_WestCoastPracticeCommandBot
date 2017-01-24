package org.usfirst.frc.team2850.robot.commands;

import org.usfirst.frc.team2850.robot.Robot;
import org.usfirst.frc.team2850.robot.subsystems.DriveLeft;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveToDistLeft extends Command {

	private double setpoint;
	
    public DriveToDistLeft() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveleft);
    }
    public DriveToDistLeft(double setpoint) {
        this.setpoint = setpoint;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	DriveLeft.leftEncoder.reset();
    	Robot.driveleft.setSetpoint(setpoint);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	System.out.println("Left Error: " + DriveLeft.leftEncoder.getDistance());
        return Math.abs(Robot.driveleft.getPosition() - setpoint) < .24;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveleft.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
