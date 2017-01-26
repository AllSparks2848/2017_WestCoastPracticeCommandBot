package org.usfirst.frc.team2850.robot.commands;

import org.usfirst.frc.team2850.robot.Robot;
import org.usfirst.frc.team2850.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GyroTurn extends Command {
	private double setpoint;
	
    public GyroTurn() {
        requires(Robot.drivetrain);
    }
    
    public GyroTurn(double setpoint) {
        this.setpoint = setpoint;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	DriveTrain.gyro.reset();
    	//Robot.drivetrain.pickPIDType("Gyro");
    	Robot.drivetrain.setSetpoint(setpoint);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	System.out.println("Gyro Angle: " + DriveTrain.gyro.getAngle());
    	return Math.abs(DriveTrain.gyro.getAngle() - setpoint) < 2.0;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
