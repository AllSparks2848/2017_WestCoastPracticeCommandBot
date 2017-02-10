package org.usfirst.frc.team2850.robot.subsystems;

/**
 * An interface that subsystems implement so Robot can easily send all data to smartdashboard.
 * 
 *  
 */
public interface SmartDashboardLogger {
  /**
   * Sends diagnostics to smartdashboard.
   */
  public void sendDataToSmartDashboard();
}
