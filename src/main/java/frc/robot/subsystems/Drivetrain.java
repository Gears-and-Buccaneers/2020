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

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  private final Gyro m_gyro = new ADXRS450_Gyro();


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
   * Get the robot's heading.
   *
   * @return The robots heading in degrees.
   */
  public double getHeading() {
    return m_gyro.getAngle();
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
  }
}
