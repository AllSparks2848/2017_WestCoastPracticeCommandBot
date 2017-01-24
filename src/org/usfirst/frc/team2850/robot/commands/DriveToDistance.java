package org.usfirst.frc.team2850.robot.commands;

import org.usfirst.frc.team2850.robot.Robot;
import org.usfirst.frc.team2850.robot.subsystems.DriveLeft;
import org.usfirst.frc.team2850.robot.subsystems.DriveRight;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveToDistance extends Command {
	private double setpoint;
	
	public DriveToDistance() {
		requires(Robot.driveright);
        requires(Robot.driveleft);
	}
    public DriveToDistance(double setpoint) {
        this.setpoint = setpoint;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	DriveLeft.leftEncoder.reset();
    	DriveRight.rightEncoder.reset();
    	Robot.driveleft.setSetpoint(setpoint);
    	Robot.driveright.setSetpoint(-setpoint);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	System.out.println("Left Error: " + DriveLeft.leftEncoder.getDistance());
    	System.out.println("Right Error: " + DriveRight.rightEncoder.getDistance());
        return Math.abs(Robot.driveleft.getPosition() - setpoint) < .24;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveleft.disable();
    	Robot.driveright.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
