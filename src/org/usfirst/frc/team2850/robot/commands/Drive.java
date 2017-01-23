package org.usfirst.frc.team2850.robot.commands;

import org.usfirst.frc.team2850.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team2850.robot.subsystems.DriveTrain;
import org.usfirst.frc.team2850.robot.subsystems.DriveTrain.*;
import org.usfirst.frc.team2850.robot.RobotMap;

/**
 *
 */
public class Drive extends Command {

    public Drive() {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("PID Drive Initialized");
    	DriveTrain.pidDriveInit(36.0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	System.out.println("PID Drive Enabled");
    	DriveTrain.pidDriveEnable();
    	System.out.println("PID Drive Driving");
    	DriveTrain.pidDrive(36.0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	System.out.println("PID Drive Finished");
    	return DriveTrain.pidDriveFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
    	System.out.println("PID Drive Ended");
    	DriveTrain.pidDriveEnd();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("PID Drive Interuptted");
    	DriveTrain.pidDriveEnd();
    }
}
