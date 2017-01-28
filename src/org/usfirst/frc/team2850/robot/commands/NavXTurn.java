package org.usfirst.frc.team2850.robot.commands;

import org.usfirst.frc.team2850.robot.Robot;
import org.usfirst.frc.team2850.robot.subsystems.GyroPIDController;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class NavXTurn extends Command {
	public double angle;
	
    public NavXTurn() {
        requires(Robot.navxturntrain);
    }
    
    public NavXTurn(double angle) {
    	this.angle = angle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	GyroPIDController.navX.reset();
    	Robot.navxturntrain.setSetpoint(this.angle);
    	Robot.navxturntrain.getPIDController().setOutputRange(-1, 1);
    	Robot.navxturntrain.getPIDController().enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	System.out.println("\n=====================");
    	System.out.println("NavX Angle: " + GyroPIDController.navX.getAngle());
    	System.out.println("PID Output: " + Robot.navxturntrain.getPIDController().get());
    	System.out.println("=====================\n");
        return Robot.navxturntrain.getPIDController().getError() < .5;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.navxturntrain.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
