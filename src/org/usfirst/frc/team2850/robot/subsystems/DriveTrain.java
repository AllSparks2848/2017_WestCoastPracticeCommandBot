package org.usfirst.frc.team2850.robot.subsystems;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.usfirst.frc.team2850.robot.FieldPosition;
import org.usfirst.frc.team2850.robot.Robot;
import org.usfirst.frc.team2850.robot.RobotMap;
import org.usfirst.frc.team2850.robot.RobotPosition;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This subsystem handles driving (duh).
 */
public class DriveTrain extends Subsystem implements SmartDashboardLogger {
  private static final Logger logger = LoggerFactory.getLogger(DriveTrain.class);

  /**
   * This is a list of all shift actions. UP and DOWN shift up and down, and TOGGLE toggles. This is
   * so that you can initialize a ShiftCommand with TOGGLE and it will always toggle.
   */
  public static enum ShiftType {
    UP, DOWN, TOGGLE
  }

  /**
   * This makes calls to getEncoderValue() readable.
   */
  public static enum DriveTrainSide {
    LEFT, RIGHT
  }

 Spark leftDrive1;
 Spark leftDrive2;
 Spark leftDrive3;
 Spark rightDrive1;
 Spark rightDrive2;
 Spark rightDrive3;
  RobotDrive robotDrive1;
  RobotDrive robotDrive2;
 
  AHRS navX;

  double positionX;
  double positionY;
  double rotation;

  double prevEncoderLeft;
  double prevEncoderRight;
  DriverStation driverStation;
  double rotationOffset;
  double lastOutputLeft = 0;
  double lastOutputRight = 0;
  Encoder rightEncoder;
  Encoder leftEncoder;

  /**
   * Creates a new drive train instance.
   */
  public DriveTrain() {
	   leftDrive1 = new Spark(RobotMap.p_leftDrive1);
		 leftDrive2 = new Spark(RobotMap.p_leftDrive2);
		 leftDrive3 = new Spark(RobotMap.p_leftDrive3);
		 rightDrive1 = new Spark(RobotMap.p_rightDrive1);
		 rightDrive2 = new Spark(RobotMap.p_rightDrive2);
	 rightDrive3 = new Spark(RobotMap.p_rightDrive3);
    logger.info("Initialize");

     rightEncoder = new Encoder(RobotMap.p_rightEncoderA, RobotMap.p_rightEncoderB, false, Encoder.EncodingType.k4X);
 leftEncoder = new Encoder(RobotMap.p_leftEncoderA, RobotMap.p_leftEncoderB, false, Encoder.EncodingType.k4X);

    robotDrive1 = new RobotDrive(leftDrive1, leftDrive2,rightDrive1,rightDrive2) {
      public void setLeftRightMotorOutputs(double leftOutput, double rightOutput) {
        super.setLeftRightMotorOutputs(leftOutput, rightOutput);
        lastOutputLeft = leftOutput;
        lastOutputRight = rightOutput;
      }
    };
    robotDrive2 = new RobotDrive(leftDrive3,rightDrive3) {
        public void setLeftRightMotorOutputs(double leftOutput, double rightOutput) {
          super.setLeftRightMotorOutputs(leftOutput, rightOutput);
          lastOutputLeft = leftOutput;
          lastOutputRight = rightOutput;
        }
      };
    robotDrive1.setMaxOutput(12.5);
    robotDrive2.setMaxOutput(12.5);

    
    navX = new AHRS(SPI.Port.kMXP);

    calibrateYaw();
  }

 

  /**
   * Sets the default command to give driver control.
   */
  public void initDefaultCommand() {
    logger.info(String.format("initDefaultCommand called, right now Robot.driveJoystickCommand=%s",
        Robot.driveJoystickCommand.toString()));
    setDefaultCommand(Robot.driveJoystickCommand);
  }

  /**
   * This method drives the robot using joystick values.
   * 
   * @param throttle is the vertical axis
   * @param turn is the horizontal axis
   */
  public void joystickDrive(double throttle, double turn) {
    logger.trace(String.format("Driving with throttle %f and turn %f", throttle, turn));
    robotDrive1.arcadeDrive(throttle, turn);
    robotDrive2.arcadeDrive(throttle, turn);
  }

  /**
   * Arcade drive but without squared inputs.
   * @param throttle is the vertical axis
   * @param turn turn is the horizontal axis
   */
  public void rawThrottleTurnDrive(double throttle, double turn) {
    logger.trace(
        String.format("Driving no squared inputs with throttle %f and turn %f", throttle, turn));
    robotDrive1.arcadeDrive(throttle, turn, false);
    robotDrive2.arcadeDrive(throttle, turn, false);
  }

  /**
   * Sets raw left and right motor values.
   * 
   * @param left The left value
   * @param right The right value
   */
  public void rawLeftRightDrive(double left, double right) {
    robotDrive1.setLeftRightMotorOutputs(left, right);
    robotDrive2.setLeftRightMotorOutputs(left, right);
  }

 
  /**
   * Returns the last output value for the motors.
   * 
   * @param side Which side to get the last output value for
   * @return The last known output value for that side
   */
  public double getLastOutput(DriveTrainSide side) {
    if (side == DriveTrainSide.LEFT) {
      return lastOutputLeft;
    } else {
      return lastOutputRight;
    }
  }




  
 

  /**
   * Gets the encoder value for the specified side.
   * 
   * @param side The side, either LEFT or RIGHT
   * @return The encoder value, in inches
   */
  public double getEncoderDistance(DriveTrainSide side) {
    // getPosition() gives revolutions, since the talons are calibrated for the ticks per
    // revolution. Multiply by wheel circumference and gearing factor to get distance in inches.
    if (side == DriveTrainSide.LEFT) {
      return leftEncoder.get()* RobotMap.GEARING_FACTOR * RobotMap.WHEEL_CIRCUMFERENCE;
    } else {
      return leftEncoder.get()* RobotMap.GEARING_FACTOR * RobotMap.WHEEL_CIRCUMFERENCE;
    }
  }

  /**
   * Sends all navx data to the dashboard.
   */
  public void dumpNavxData() {
    SmartDashboard.putBoolean("IMU_Connected", navX.isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", navX.isCalibrating());
    SmartDashboard.putNumber("IMU_Yaw", navX.getYaw());
    SmartDashboard.putNumber("IMU_Pitch", navX.getPitch());
    SmartDashboard.putNumber("IMU_Roll", navX.getRoll());

    SmartDashboard.putNumber("IMU_CompassHeading", navX.getCompassHeading());

    SmartDashboard.putNumber("IMU_FusedHeading", navX.getFusedHeading());

    SmartDashboard.putNumber("IMU_TotalYaw", navX.getAngle());
    SmartDashboard.putNumber("IMU_YawRateDPS", navX.getRate());

    SmartDashboard.putNumber("IMU_Accel_X", navX.getWorldLinearAccelX());
    SmartDashboard.putNumber("IMU_Accel_Y", navX.getWorldLinearAccelY());
    SmartDashboard.putBoolean("IMU_IsMoving", navX.isMoving());
    SmartDashboard.putBoolean("IMU_IsRotating", navX.isRotating());

    SmartDashboard.putNumber("Velocity_X", navX.getVelocityX());
    SmartDashboard.putNumber("Velocity_Y", navX.getVelocityY());
    SmartDashboard.putNumber("Displacement_X", navX.getDisplacementX());
    SmartDashboard.putNumber("Displacement_Y", navX.getDisplacementY());

    SmartDashboard.putNumber("RawGyro_X", navX.getRawGyroX());
    SmartDashboard.putNumber("RawGyro_Y", navX.getRawGyroY());
    SmartDashboard.putNumber("RawGyro_Z", navX.getRawGyroZ());
    SmartDashboard.putNumber("RawAccel_X", navX.getRawAccelX());
    SmartDashboard.putNumber("RawAccel_Y", navX.getRawAccelY());
    SmartDashboard.putNumber("RawAccel_Z", navX.getRawAccelZ());
    SmartDashboard.putNumber("RawMag_X", navX.getRawMagX());
    SmartDashboard.putNumber("RawMag_Y", navX.getRawMagY());
    SmartDashboard.putNumber("RawMag_Z", navX.getRawMagZ());
    SmartDashboard.putNumber("IMU_Temp_C", navX.getTempC());

    AHRS.BoardYawAxis yawAxis = navX.getBoardYawAxis();
    SmartDashboard.putString("YawAxisDirection", yawAxis.up ? "Up" : "Down");
    SmartDashboard.putNumber("YawAxis", yawAxis.board_axis.getValue());

    SmartDashboard.putString("FirmwareVersion", navX.getFirmwareVersion());


    SmartDashboard.putNumber("QuaternionW", navX.getQuaternionW());
    SmartDashboard.putNumber("QuaternionX", navX.getQuaternionX());
    SmartDashboard.putNumber("QuaternionY", navX.getQuaternionY());
    SmartDashboard.putNumber("QuaternionZ", navX.getQuaternionZ());

    SmartDashboard.putNumber("IMU_Byte_Count", navX.getByteCount());
    SmartDashboard.putNumber("IMU_Update_Count", navX.getUpdateCount());
  }

  /**
   * Sends all navx and talon data to the dashboard.
   */
  public void sendDataToSmartDashboard() {
    dumpNavxData();

    



    SmartDashboard.putNumber("Corrected_Yaw", rotation);

    SmartDashboard.putNumber("Encoder_Left", getEncoderDistance(DriveTrainSide.LEFT));
    SmartDashboard.putNumber("Encoder_Right", getEncoderDistance(DriveTrainSide.RIGHT));

   

    
  }

  public RobotPosition getRobotPosition() {
    return new RobotPosition(positionX, positionY, rotation);
  }

  public void setPosition(FieldPosition fieldPos) {
    positionX = fieldPos.getX();
    positionY = fieldPos.getY();
  }

  /**
   * Updates the dead reckoning for our current position.
   */
  public void updatePosition() {
    rotation = fixDegrees(navX.getYaw() + rotationOffset);

    double encoderLeft = getEncoderDistance(DriveTrainSide.LEFT);
    double encoderRight = getEncoderDistance(DriveTrainSide.RIGHT);

    double deltaEncoderLeft = encoderLeft - prevEncoderLeft;
    double deltaEncoderRight = encoderRight - prevEncoderRight;

    double deltaInches = (deltaEncoderLeft + deltaEncoderRight) / 2;

    positionX = positionX + Math.cos(Math.toRadians(rotation)) * deltaInches;
    positionY = positionY + Math.sin(Math.toRadians(rotation)) * deltaInches;

    prevEncoderLeft = encoderLeft;
    prevEncoderRight = encoderRight;
  }

  public double getYaw() {
    return rotation;
  }

  /**
   * Sets initial yaw based on where our starting position is.
   */
  public void calibrateYaw() {
    if (DriverStation.getInstance().getAlliance() == DriverStation.Alliance.Blue) {
      rotationOffset = -90.0;
    } else {
      rotationOffset = 90.0;
    }
  }

  /**
   * Zeroes the NavX.
   */
  public void resetNavX() {
    navX.reset();
    navX.resetDisplacement();
  }

  public static double fixDegrees(double angle) {
    return ((angle % 360) + 360) % 360;
  }
}