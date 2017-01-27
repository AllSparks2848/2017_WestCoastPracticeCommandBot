package org.usfirst.frc.team2850.robot.commands;

import org.usfirst.frc.team2850.robot.Robot;
import org.usfirst.frc.team2850.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;

public class GyroTurn extends Command {
	private double setpoint;
	private double pGyro = .008;
	private double gyroError;
	private double clippedOutput;
    public GyroTurn(double setpoint) {
        requires(Robot.drivetrain);
        this.setpoint = setpoint;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	DriveTrain.gyro.reset();
    	gyroError = setpoint - DriveTrain.gyro.getAngle();
    	clippedOutput = gyroError*pGyro;
    	if(clippedOutput>.8) {
    		clippedOutput = .8;
    	}
    	if(clippedOutput<-.8) {
    		clippedOutput = -.8;
    	}
    	Robot.drivetrain.tankDrive(clippedOutput, -clippedOutput);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	gyroError = setpoint - DriveTrain.gyro.getAngle();
    	Robot.drivetrain.tankDrive(clippedOutput, -clippedOutput);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	System.out.println("Gyro Error: " + gyroError);
        return Math.abs(gyroError) < 2.5;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.disable();
    	Robot.drivetrain.tankDrive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
