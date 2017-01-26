package org.usfirst.frc.team2850.robot.commands;

import org.usfirst.frc.team2850.robot.Robot;
import org.usfirst.frc.team2850.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;

public class DriveToDistance extends Command {
	private double setpoint;
	
	public DriveToDistance() {
		requires(Robot.drivetrain);
	}
    public DriveToDistance(double setpoint) {
        this.setpoint = setpoint;
    }

    protected void initialize() {
    	DriveTrain.leftEncoder.reset();
    	DriveTrain.rightEncoder.reset();
    	//Robot.drivetrain.pickPIDType("Encoder");
    	Robot.drivetrain.setSetpoint(setpoint);
    }

    protected void execute() {
    }

    protected boolean isFinished() {
    	System.out.println("Left Dist: " + DriveTrain.leftEncoder.getDistance());
    	System.out.println("Right Dist: " + DriveTrain.rightEncoder.getDistance());
    	System.out.println("Avg Dist: " + (DriveTrain.leftEncoder.getDistance()+DriveTrain.rightEncoder.getDistance())/2);
    	System.out.println("Position: " + Robot.drivetrain.getPosition());
    	System.out.println("Error: " + Robot.drivetrain.getPIDController().getError());
    	System.out.println("P term: " + Robot.drivetrain.getPIDController().getP());
    	System.out.println("PID Out: " + Robot.drivetrain.getPIDController().get());
        return Math.abs(Robot.drivetrain.getPosition() - setpoint) < .75;
    }

    protected void end() {
    	Robot.drivetrain.disable();
    }

    protected void interrupted() {
    }
}
