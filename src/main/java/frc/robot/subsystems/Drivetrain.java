/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
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

import com.ctre.phoenix.music.Orchestra;

import java.util.ArrayList;

import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
//import io.github.oblarg.oblog.annotations.Log;

public class Drivetrain extends SubsystemBase {
  // left side motors
  private final WPI_TalonFX leftMaster = new WPI_TalonFX(DriveConstants.kLeftMaster);
  private final TalonFX leftSlave = new TalonFX(DriveConstants.kLeftSlave);

  // right side motors
  private final WPI_TalonFX rightMaster = new WPI_TalonFX(DriveConstants.kRightMaster);
  private final TalonFX rightSlave = new TalonFX(DriveConstants.kRightSlave);

  // Config Objects for motor controllers
  private final TalonFXConfiguration leftMasterConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration rightMasterConfig = new TalonFXConfiguration();

  // Invert Directions for Left and Right
	private final TalonFXInvertType leftInvert = TalonFXInvertType.CounterClockwise; //Same as invert = "false"
	private final TalonFXInvertType rightInvert = TalonFXInvertType.Clockwise; //Same as invert = "true"

  // the drivebase
  private final DifferentialDrive m_drive = new DifferentialDrive(leftMaster, rightMaster);

  //the gyro
  private final Gyro m_gyro = new ADIS16448_IMU();

  //music!
  private Orchestra orchestra;
  private ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();

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

  private boolean swapDrive;

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

    /** Feedback Sensor Configuration */
		
		/* Configure the left Talon's selected sensor as local Integrated Sensor */
		leftMasterConfig.primaryPID.selectedFeedbackSensor =	TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();	// Local Feedback Source

    /* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
		rightMasterConfig.remoteFilter0.remoteSensorDeviceID = leftMaster.getDeviceID(); // Device ID of Source
    rightMasterConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; // Remote Feedback Source

    /* Now that the Left sensor can be used by the master Talon,
		 * set up the Left (Aux) and Right (Master) distance into a single
		 * Robot distance as the Master's Selected Sensor 0. */
    setRobotDistanceConfigs(rightInvert, rightMasterConfig);

    /* Setup difference signal to be used for turn when performing Drive Straight with encoders */
    setRobotTurnConfigs(rightInvert, rightMasterConfig);
    
    /* Configure neutral deadband */
		rightMasterConfig.neutralDeadband = Constants.DriveConstants.kNeutralDeadband;
    leftMasterConfig.neutralDeadband = Constants.DriveConstants.kNeutralDeadband;
    
    /* Motion Magic Configurations */
		rightMasterConfig.motionAcceleration = 2000;
		rightMasterConfig.motionCruiseVelocity = 2000;

		/**
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		leftMasterConfig.peakOutputForward = +1.0;
		leftMasterConfig.peakOutputReverse = -1.0;
		rightMasterConfig.peakOutputForward = +1.0;
		rightMasterConfig.peakOutputReverse = -1.0;

		/* FPID Gains for distance servo */
		rightMasterConfig.slot0.kP = Constants.DriveConstants.kGains_Distanc.kP;
		rightMasterConfig.slot0.kI = Constants.DriveConstants.kGains_Distanc.kI;
		rightMasterConfig.slot0.kD = Constants.DriveConstants.kGains_Distanc.kD;
		rightMasterConfig.slot0.kF = Constants.DriveConstants.kGains_Distanc.kF;
		rightMasterConfig.slot0.integralZone = Constants.DriveConstants.kGains_Distanc.kIzone;
		rightMasterConfig.slot0.closedLoopPeakOutput = Constants.DriveConstants.kGains_Distanc.kPeakOutput;
		rightMasterConfig.slot0.allowableClosedloopError = 0;

		/* FPID Gains for turn servo */
		rightMasterConfig.slot1.kP = Constants.DriveConstants.kGains_Turning.kP;
		rightMasterConfig.slot1.kI = Constants.DriveConstants.kGains_Turning.kI;
		rightMasterConfig.slot1.kD = Constants.DriveConstants.kGains_Turning.kD;
		rightMasterConfig.slot1.kF = Constants.DriveConstants.kGains_Turning.kF;
		rightMasterConfig.slot1.integralZone = Constants.DriveConstants.kGains_Turning.kIzone;
		rightMasterConfig.slot1.closedLoopPeakOutput = Constants.DriveConstants.kGains_Turning.kPeakOutput;
		rightMasterConfig.slot1.allowableClosedloopError = 0;

		/**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		rightMasterConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		rightMasterConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
		rightMasterConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
		rightMasterConfig.slot3.closedLoopPeriod = closedLoopTimeMs;

		leftMaster.configAllSettings(leftMasterConfig);
		rightMaster.configAllSettings(rightMasterConfig);
		
		/* Configure output and sensor direction */
		leftMaster.setInverted(leftInvert);
		rightMaster.setInverted(rightInvert);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _leftMaster.setSensorPhase(true);
        // _rightMaster.setSensorPhase(true);
		
		/* Set status frame periods to ensure we don't have stale data */
		rightMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.DriveConstants.kTimeoutMs);
		rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.DriveConstants.kTimeoutMs);
		rightMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.DriveConstants.kTimeoutMs);
		rightMaster.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, Constants.DriveConstants.kTimeoutMs);
		leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 25, Constants.DriveConstants.kTimeoutMs);

		rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
    zeroSensors();
    
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftMaster.setNeutralMode(NeutralMode.Brake);

    instruments.add(leftMaster);
    instruments.add(leftSlave);
    instruments.add(rightMaster);
    instruments.add(rightSlave);

    orchestra = new Orchestra(instruments);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    orchestra.loadMusic("cleanup.chrp");

    m_drive.setSafetyEnabled(false);

    m_drive.setRightSideInverted(false);

    swapDrive = false;
    SmartDashboard.putBoolean("drive swapped?", swapDrive);

  }

  public void arcadeDrive(double fwd, double rot) {
    if(swapDrive){
      m_drive.arcadeDrive(-fwd, rot+0.125);
    }
    else{
      m_drive.arcadeDrive(fwd, rot+0.125, true); // Squaring values
    }
  }

  public void drive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  public void arcadeDriveCTRE(double forward, double turn){
    leftMaster.set(TalonFXControlMode.PercentOutput, -(forward), DemandType.ArbitraryFeedForward, +turn);
    rightMaster.set(TalonFXControlMode.PercentOutput, -(forward), DemandType.ArbitraryFeedForward, -turn);
    m_drive.feed();
  }

  public void arcadeDriveWithFeedforwardPID(double fwdSetpoint, double rotSetpoint){

  }

  public boolean getMotionMagicFinished(){
    if ((leftMaster.getClosedLoopError() < +Constants.DriveConstants.kErrThreshold && leftMaster.getClosedLoopError() > -Constants.DriveConstants.kErrThreshold)
    &&(rightMaster.getClosedLoopError() < +Constants.DriveConstants.kErrThreshold && rightMaster.getClosedLoopError() > -Constants.DriveConstants.kErrThreshold)){
      return true;
    }

    return false;
  }

  public void stop(){
    leftMaster.set(ControlMode.PercentOutput, 0);
    rightMaster.set(ControlMode.PercentOutput, 0);
  }

  public void driveStraightMotionMagic(double rotations, double target_turn){
    /* Determine which slot affects which PID */
    rightMaster.selectProfileSlot(Constants.DriveConstants.kSlot_Distanc, Constants.DriveConstants.PID_PRIMARY);
    rightMaster.selectProfileSlot(Constants.DriveConstants.kSlot_Turning, Constants.DriveConstants.PID_TURN);

    double target_sensorUnits = rotations * Constants.DriveConstants.kSensorUnitsPerRotation;
    /* Configured for MotionMagic on Integrated Sensors' Sum and Auxiliary PID on Integrated Sensors' Difference */
			rightMaster.set(TalonFXControlMode.MotionMagic, target_sensorUnits, DemandType.AuxPID, target_turn);
			leftMaster.follow(rightMaster, FollowerType.AuxOutput1);
  }


  public void log(){
    SmartDashboard.putNumber("encoder distance in inches", getAverageEncoderDistanceInInches());
  }

  public void playMusic(){
    orchestra.play();
  }

  /** Zero integrated encoders on Talon */
  void zeroSensors() {
    leftMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.DriveConstants.kTimeoutMs);
    rightMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.DriveConstants.kTimeoutMs);
    System.out.println("[Integrated Sensors] All sensors are zeroed.\n");
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftEncoder() {
    return leftMaster.getSensorCollection().getIntegratedSensorPosition();
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public double getRightEncoder() {
    return rightMaster.getSensorCollection().getIntegratedSensorPosition();
  }

  public void resetEncoders() {
    leftMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.DriveConstants.kTimeoutMs);
    rightMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.DriveConstants.kTimeoutMs);
  }

  public void calibrateGyro(){
    m_gyro.calibrate();
  }

  public void resetAllSensors() {
    leftMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.DriveConstants.kTimeoutMs);
    rightMaster.getSensorCollection().setIntegratedSensorPosition(0, Constants.DriveConstants.kTimeoutMs);
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
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), leftMaster.getSensorCollection().getIntegratedSensorPosition(),
                      rightMaster.getSensorCollection().getIntegratedSensorPosition());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void toggleSwap(){
    if(swapDrive){
      swapDrive = false;
      SmartDashboard.putBoolean("drive swapped?", swapDrive);
    }
    else{
      swapDrive = true;
      SmartDashboard.putBoolean("drive swapped?", swapDrive);
    }
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftMaster.getSensorCollection().getIntegratedSensorVelocity(), rightMaster.getSensorCollection().getIntegratedSensorVelocity());
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

  /** 
	 * Determines if SensorSum or SensorDiff should be used 
	 * for combining left/right sensors into Robot Distance.  
	 * 
	 * Assumes Aux Position is set as Remote Sensor 0.  
	 * 
	 * configAllSettings must still be called on the master config
	 * after this function modifies the config values. 
	 * 
	 * @param masterInvertType Invert of the Master Talon
	 * @param masterConfig Configuration object to fill
	 */
  void setRobotDistanceConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig){
		/**
		 * Determine if we need a Sum or Difference.
		 * 
		 * The auxiliary Talon FX will always be positive
		 * in the forward direction because it's a selected sensor
		 * over the CAN bus.
		 * 
		 * The master's native integrated sensor may not always be positive when forward because
		 * sensor phase is only applied to *Selected Sensors*, not native
		 * sensor sources.  And we need the native to be combined with the 
		 * aux (other side's) distance into a single robot distance.
		 */

		/* THIS FUNCTION should not need to be modified. 
		   This setup will work regardless of whether the master
		   is on the Right or Left side since it only deals with
		   distance magnitude.  */

		/* Check if we're inverted */
		if (masterInvertType == TalonFXInvertType.Clockwise){
			/* 
				If master is inverted, that means the integrated sensor
				will be negative in the forward direction.
				If master is inverted, the final sum/diff result will also be inverted.
				This is how Talon FX corrects the sensor phase when inverting 
				the motor direction.  This inversion applies to the *Selected Sensor*,
				not the native value.
				Will a sensor sum or difference give us a positive total magnitude?
				Remember the Master is one side of your drivetrain distance and 
				Auxiliary is the other side's distance.
					Phase | Term 0   |   Term 1  | Result
				Sum:  -1 *((-)Master + (+)Aux   )| NOT OK, will cancel each other out
				Diff: -1 *((-)Master - (+)Aux   )| OK - This is what we want, magnitude will be correct and positive.
				Diff: -1 *((+)Aux    - (-)Master)| NOT OK, magnitude will be correct but negative
			*/

			masterConfig.diff0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
			masterConfig.diff1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();   //Aux Selected Sensor
			masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Diff0 - Diff1
		} else {
			/* Master is not inverted, both sides are positive so we can sum them. */
			masterConfig.sum0Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();    //Aux Selected Sensor
			masterConfig.sum1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local IntegratedSensor
			masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1
		}

		/* Since the Distance is the sum of the two sides, divide by 2 so the total isn't double
		   the real-world value */
		masterConfig.primaryPID.selectedFeedbackCoefficient = 0.5;
   }
   
   /** 
	 * Determines if SensorSum or SensorDiff should be used 
	 * for combining left/right sensors into Robot Distance.  
	 * 
	 * Assumes Aux Position is set as Remote Sensor 0.  
	 * 
	 * configAllSettings must still be called on the master config
	 * after this function modifies the config values. 
	 * 
	 * @param masterInvertType Invert of the Master Talon
	 * @param masterConfig Configuration object to fill
	 */
	void setRobotTurnConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig){
		/**
		 * Determine if we need a Sum or Difference.
		 * 
		 * The auxiliary Talon FX will always be positive
		 * in the forward direction because it's a selected sensor
		 * over the CAN bus.
		 * 
		 * The master's native integrated sensor may not always be positive when forward because
		 * sensor phase is only applied to *Selected Sensors*, not native
		 * sensor sources.  And we need the native to be combined with the 
		 * aux (other side's) distance into a single robot heading.
		 */

		/* THIS FUNCTION should not need to be modified. 
		   This setup will work regardless of whether the master
		   is on the Right or Left side since it only deals with
		   heading magnitude.  */

		/* Check if we're inverted */
		if (masterInvertType == TalonFXInvertType.Clockwise){
			/* 
				If master is inverted, that means the integrated sensor
				will be negative in the forward direction.
				If master is inverted, the final sum/diff result will also be inverted.
				This is how Talon FX corrects the sensor phase when inverting 
				the motor direction.  This inversion applies to the *Selected Sensor*,
				not the native value.
				Will a sensor sum or difference give us a positive heading?
				Remember the Master is one side of your drivetrain distance and 
				Auxiliary is the other side's distance.
					Phase | Term 0   |   Term 1  | Result
				Sum:  -1 *((-)Master + (+)Aux   )| OK - magnitude will cancel each other out
				Diff: -1 *((-)Master - (+)Aux   )| NOT OK - magnitude increases with forward distance.
				Diff: -1 *((+)Aux    - (-)Master)| NOT OK - magnitude decreases with forward distance
			*/

			masterConfig.sum0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
			masterConfig.sum1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();   //Aux Selected Sensor
			masterConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1

			/*
				PID Polarity
				With the sensor phasing taken care of, we now need to determine if the PID polarity is in the correct direction
				This is important because if the PID polarity is incorrect, we will run away while trying to correct
				Will inverting the polarity give us a positive counterclockwise heading?
				If we're moving counterclockwise(+), and the master is on the right side and inverted,
				it will have a negative velocity and the auxiliary will have a negative velocity
				 heading = right + left
				 heading = (-) + (-)
				 heading = (-)
				Let's assume a setpoint of 0 heading.
				This produces a positive error, in order to cancel the error, the right master needs to
				drive backwards. This means the PID polarity needs to be inverted to handle this
				
				Conversely, if we're moving counterclwise(+), and the master is on the left side and inverted,
				it will have a positive velocity and the auxiliary will have a positive velocity.
				 heading = right + left
				 heading = (+) + (+)
				 heading = (+)
				Let's assume a setpoint of 0 heading.
				This produces a negative error, in order to cancel the error, the left master needs to
				drive forwards. This means the PID polarity needs to be inverted to handle this
			*/
			masterConfig.auxPIDPolarity = true;
		} else {
			/* Master is not inverted, both sides are positive so we can diff them. */
			masterConfig.diff0Term = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice();    //Aux Selected Sensor
			masterConfig.diff1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local IntegratedSensor
			masterConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Sum0 + Sum1
			/* With current diff terms, a counterclockwise rotation results in negative heading with a right master */
			masterConfig.auxPIDPolarity = true;
		}
		/**
		 * Heading units should be scaled to ~4000 per 360 deg, due to the following limitations...
		 * - Target param for aux PID1 is 18bits with a range of [-131072,+131072] units.
		 * - Target for aux PID1 in motion profile is 14bits with a range of [-8192,+8192] units.
		 *  ... so at 3600 units per 360', that ensures 0.1 degree precision in firmware closed-loop
		 *  and motion profile trajectory points can range +-2 rotations.
		 */
		masterConfig.auxiliaryPID.selectedFeedbackCoefficient = Constants.DriveConstants.kEncoderUnitsPerRotation / Constants.DriveConstants.kEncoderUnitsPerRotation;
	 }
}
