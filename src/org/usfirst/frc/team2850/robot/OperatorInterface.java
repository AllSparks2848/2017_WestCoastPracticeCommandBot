package org.usfirst.frc.team2850.robot;


import org.usfirst.frc.team2850.robot.commands.CameraFeedCommand;
import org.usfirst.frc.team2850.robot.commands.DriveDistanceCommand;
import org.usfirst.frc.team2850.robot.commands.DriveToGearCommand;
import org.usfirst.frc.team2850.robot.commands.TurnCommand;
import org.usfirst.frc.team2850.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator interface to the commands
 * and command groups that allow control of the robot.
 */
public class OperatorInterface {
 Joystick xbox;

  /**
   * This is where we set up the operator interface.
   */
  public OperatorInterface() {
    xbox = new Joystick(0);

   

    JoystickButton xboxRightBumper = new JoystickButton(xbox, 6);
    xboxRightBumper.whenPressed(new DriveToGearCommand());

  

    SmartDashboard.putData(new TurnCommand(180));
    SmartDashboard.putData(new DriveDistanceCommand(12 * 5));
    SmartDashboard.putData(new DriveToGearCommand());
    
   
    
    SmartDashboard.putData(new CameraFeedCommand(Vision.StreamType.TOGGLE));
    
    SmartDashboard.putData(new InstantCommand("ResetYaw") {
      @Override
      public void execute() {
        Robot.driveTrain.resetNavX();
      }
    });
  }

  public double getThrottle() {
    return xbox.getRawAxis(1);
  }

  public double getTurn() {
    return xbox.getRawAxis(4);
  }

  /**
   * Every command that implements automatic behavior should check this in isFinished(). This method
   * returns true if
   * <ul>
   * <li>The left (throttle) stick button is pressed, or</li>
   * <li>Throttle is applied beyond 0.5</li>
   * </ul>
   * 
   * @return True if the above conditions are met.
   */
  public boolean isCancelled() {
    // just to be safe. The controller could fall down or something.
    if (DriverStation.getInstance().isAutonomous()) {
      return false;
    }
    return getThrottle() > 0.5 || getThrottle() < -0.5;
  }
}