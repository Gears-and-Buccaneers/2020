/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants.TurretConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  private static final TalonSRX m_TurretMotor = new TalonSRX(TurretConstants.kTurretMotor);

  public Turret() {
    //reset talons to factory defaults just in case of a swap or a setting changed in phoenix tuner
    m_TurretMotor.configFactoryDefault(TurretConstants.kTimeoutMs);

    //makes sure that the motor brakes when no input is applied so that it hold position.
    m_TurretMotor.setNeutralMode(NeutralMode.Brake);

    //sets the aboslute encoder as the sensor input for the SRX
    m_TurretMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, TurretConstants.kPIDLoopIdx, TurretConstants.kTimeoutMs);

    //tells the SRX that the sensor is reading in the same direction as the output
    m_TurretMotor.setSensorPhase(false);

    // Set relevant frame periods to be at least as fast as periodic rate
		m_TurretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, TurretConstants.kTimeoutMs);
    m_TurretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TurretConstants.kTimeoutMs);
    
    // Set the peak and nominal outputs
		m_TurretMotor.configNominalOutputForward(0, TurretConstants.kTimeoutMs);
		m_TurretMotor.configNominalOutputReverse(0, TurretConstants.kTimeoutMs);
		m_TurretMotor.configPeakOutputForward(1, TurretConstants.kTimeoutMs);
    m_TurretMotor.configPeakOutputReverse(-1, TurretConstants.kTimeoutMs);
    
    // Set Motion Magic gains in slot0 - see documentation
    m_TurretMotor.selectProfileSlot(TurretConstants.kSlotIdx, TurretConstants.kPIDLoopIdx);
    m_TurretMotor.config_kF(TurretConstants.kSlotIdx, TurretConstants.kF);
    m_TurretMotor.config_kP(TurretConstants.kSlotIdx, TurretConstants.kP);
    m_TurretMotor.config_kI(TurretConstants.kSlotIdx, TurretConstants.kI);
    m_TurretMotor.config_kD(TurretConstants.kSlotIdx, TurretConstants.kD);
    
    /* Set acceleration and vcruise velocity - see documentation */
		m_TurretMotor.configMotionCruiseVelocity(TurretConstants.kCuriseVelocity, TurretConstants.kTimeoutMs);
		m_TurretMotor.configMotionAcceleration(TurretConstants.kAccelerationVelocity, TurretConstants.kTimeoutMs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void zero(){
    m_TurretMotor.set(ControlMode.MotionMagic, 0); //moves the turret to 0 degrees
  }

  public void turnToAngle(double angle){
    m_TurretMotor.set(ControlMode.MotionMagic, angle); //move the turret to the desired angle
  }
}
