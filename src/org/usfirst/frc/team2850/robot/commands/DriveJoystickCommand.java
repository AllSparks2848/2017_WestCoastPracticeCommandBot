package org.usfirst.frc.team2850.robot.commands;


import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.usfirst.frc.team2850.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Command that lets you use the joysticks to drive.
 */
public class DriveJoystickCommand extends Command {
  private static final Logger logger = LoggerFactory.getLogger(DriveJoystickCommand.class);
  
  /**
   * Creates a DriveJoystickCommand.
   */
  public DriveJoystickCommand() {
    requires(Robot.driveTrain);
  }

  protected void initialize() {
    logger.info("Initialize");
  }

  protected void execute() {
    Robot.driveTrain.joystickDrive(Robot.operatorInterface.getThrottle(),
        Robot.operatorInterface.getTurn());
  }

  protected boolean isFinished() {
    return false;
  }

  protected void end() {
    // never called
    logger.error("end() called, Scheduler probably screwed up");
  }

  protected void interrupted() {
    // not warn because we expect this to be interrupted by auto driving later
    logger.info("Interrupted");
    Robot.driveTrain.joystickDrive(0, 0);
  }
}
