package org.usfirst.frc.team2850.robot.commands;

import org.usfirst.frc.team2850.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class BackAllDown extends Command {

    public BackAllDown() {
    	requires(Robot.drivetrain);
    }

    protected void initialize() {
    	Robot.drivetrain.backAllDown();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
