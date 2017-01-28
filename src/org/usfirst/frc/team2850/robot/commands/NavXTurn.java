package org.usfirst.frc.team2850.robot.commands;

import org.usfirst.frc.team2850.robot.Robot;
import org.usfirst.frc.team2850.robot.subsystems.GyroPIDController;
import org.usfirst.frc.team2850.robot.subsystems.GyroPIDController.*;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class NavXTurn extends Command {
	public double angle;
	private boolean finished = false;
	
    public NavXTurn() {
        requires(Robot.navxturntrain);
    }
    
    public NavXTurn(double angle) {
    	this.angle = angle;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	GyroPIDController.navX.reset();
    	finished = false;
    	Robot.navxturntrain.setSetpoint(this.angle);
    	System.out.println("Initial NavX Angle: " + GyroPIDController.navX.getAngle() + "***********************************************");
    	Robot.navxturntrain.getPIDController().setOutputRange(-.8, .8);
    	Robot.navxturntrain.getPIDController().enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Math.abs(Robot.navxturntrain.getPIDController().get()) < .1 && Math.abs(Robot.navxturntrain.getPIDController().get()) > 0) {
    		if (Robot.navxturntrain.getPIDController().get() < 0) {
    			Robot.navxturntrain.usePIDOutput(-.5);
    		} 
    		else {
    			Robot.navxturntrain.usePIDOutput(.5);
    		}
    		
    	}
    	if (!(Math.abs(Robot.navxturntrain.getPIDController().getError()) < .5)) {
    		finished = false;
    		System.out.println("\n=====================");
        	System.out.println("NavX Angle: " + GyroPIDController.navX.getAngle());
        	System.out.println("Error: " + Robot.navxturntrain.getPIDController().getError());
        	System.out.println("Calculated Error: " + Math.abs(Robot.navxturntrain.getSetpoint() - GyroPIDController.navX.getAngle()));
        	System.out.println("PID Output: " + Robot.navxturntrain.getPIDController().get());
        	System.out.println("=====================\n");
    	}
    	else {
    		finished = true;
     		System.out.println("\n=====================");
        	System.out.println("FINAL NavX Angle: " + GyroPIDController.navX.getAngle());
        	System.out.println("FINAL Error: " + Robot.navxturntrain.getPIDController().getError());
        	System.out.println("FINAL Calculated Error: " + Math.abs(Robot.navxturntrain.getSetpoint() - GyroPIDController.navX.getAngle()));
        	System.out.println("FINAL PID Output: " + Robot.navxturntrain.getPIDController().get());
        	System.out.println("=====================\n");
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finished;
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
