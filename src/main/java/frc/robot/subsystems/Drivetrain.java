/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
//import io.github.oblarg.oblog.annotations.Log;

public class Drivetrain extends SubsystemBase {
  // left side motors
  private final WPI_TalonFX leftMaster = new WPI_TalonFX(DriveConstants.kLeftMaster);
  private final TalonFX leftSlave = new TalonFX(DriveConstants.kLeftSlave);
  private final TalonFXSensorCollection leftSensors = new TalonFXSensorCollection(leftSlave); //need in order to get encoder values from talon fx
  // right side motors
  private final WPI_TalonFX rightMaster = new WPI_TalonFX(DriveConstants.kRightMaster);
  private final TalonFX rightSlave = new TalonFX(DriveConstants.kRightSlave);
  private final TalonFXSensorCollection rightSensors = new TalonFXSensorCollection(rightSlave); //need in order to get encoder values from talon fx

  // the drivebase
  private final DifferentialDrive m_drive = new DifferentialDrive(leftMaster, rightMaster);

  //the gyro
  private final Gyro m_gyro = new ADIS16448_IMU();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  //kinematics and path following stuff
  public static final double kTrackwidthMeters = 0.69;
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

  // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
  // These characterization values MUST be determined either experimentally or theoretically
  // for *your* robot's drive.
  // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
  // values for your robot.
  public static final double ksVolts = 0.22;
  public static final double kvVoltSecondsPerMeter = 1.98;
  public static final double kaVoltSecondsSquaredPerMeter = 0.2;

  // Example value only - as above, this must be tuned for your drive!
  public static final double kPDriveVel = 8.5;

  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;

  // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  public Drivetrain() {
    //reset talons to factory defaults just in case of a swap or a setting changed in phoenix tuner
    leftMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();
    rightMaster.configFactoryDefault();
    rightSlave.configFactoryDefault();

    // only need to talk to one motor per side
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    // all of this is to prevent battery brownouts
    leftMaster.configVoltageCompSaturation(DriveConstants.kVoltageCompLevel);
    leftMaster.enableVoltageCompensation(true);
    leftMaster.configOpenloopRamp(DriveConstants.kRampCoefficient);
    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftSlave.configVoltageCompSaturation(DriveConstants.kVoltageCompLevel);
    leftSlave.enableVoltageCompensation(true);
    leftSlave.configOpenloopRamp(DriveConstants.kRampCoefficient);
    leftSlave.setNeutralMode(NeutralMode.Brake);
    rightMaster.configVoltageCompSaturation(DriveConstants.kVoltageCompLevel);
    rightMaster.enableVoltageCompensation(true);
    rightMaster.configOpenloopRamp(DriveConstants.kRampCoefficient);    
    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightSlave.configVoltageCompSaturation(DriveConstants.kVoltageCompLevel);
    rightSlave.enableVoltageCompensation(true);
    rightSlave.configOpenloopRamp(DriveConstants.kRampCoefficient);
    rightSlave.setNeutralMode(NeutralMode.Brake);

    rightSlave.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);
    leftSlave.configIntegratedSensorAbsoluteRange(AbsoluteSensorRange.Unsigned_0_to_360);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void arcadeDriveWithFeedforwardPID(double fwdSetpoint, double rotSetpoint){

  }


  public void log(){
    SmartDashboard.putNumber("encoder distance in inches", getAverageEncoderDistanceInInches());
  }



  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftEncoder() {
    return leftSensors.getIntegratedSensorPosition();
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public double getRightEncoder() {
    return rightSensors.getIntegratedSensorPosition();
  }

  public void resetEncoders() {
    leftSensors.setIntegratedSensorPosition(0,0); //resets encoders and doesn't check if it's done properly
    rightSensors.setIntegratedSensorPosition(0,0);
  }

  public void resetAllSensors() {
    leftSensors.setIntegratedSensorPosition(0,0); //resets encoders and doesn't check if it's done properly
    rightSensors.setIntegratedSensorPosition(0,0);
    m_gyro.reset();
  }

  public double getAverageEncoderDistance() {
    return ((getLeftEncoder() + getRightEncoder()) / 2.0);
  }

  public double getAverageEncoderDistanceInInches() {
    return ((getLeftEncoder() + getRightEncoder()) / 2.0)/2048/10.75; //need to check math here, but this seems right
  }



  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), leftSensors.getIntegratedSensorPosition(),
                      rightSensors.getIntegratedSensorPosition());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftSensors.getIntegratedSensorVelocity(), rightSensors.getIntegratedSensorVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMaster.setVoltage(leftVolts);
    rightMaster.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
