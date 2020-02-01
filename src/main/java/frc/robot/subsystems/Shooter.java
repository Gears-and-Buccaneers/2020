/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */
  private static final WPI_TalonSRX m_ShooterMaster = new WPI_TalonSRX(ShooterConstants.kShooterMaster);
  private static final WPI_TalonSRX m_ShooterSlave = new WPI_TalonSRX(ShooterConstants.kShooterSlave);

  public Shooter() {
    //reset talons to factory defaults just in case of a swap or a setting changed in phoenix tuner
    m_ShooterMaster.configFactoryDefault();
    m_ShooterSlave.configFactoryDefault();

    //makes sure that the motors coast and do not brake so it causes less stress on the motors because it is a shooter
    m_ShooterMaster.setNeutralMode(NeutralMode.Coast);
    m_ShooterSlave.setNeutralMode(NeutralMode.Coast);

    //make one side (non-encoder motor) slave and reverse it
    m_ShooterSlave.setInverted(true);
    m_ShooterSlave.follow(m_ShooterMaster);

    //set the sensor for the master shooter to be the mag encoder
    m_ShooterMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_ShooterMaster.setSensorPhase(false); //ensures that encoder is reading in the same direction as motor
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
