/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import frc.robot.Constants.CANIfierConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
  private final CANifier canifier = new CANifier(CANIfierConstants.kCanifier1);
  
  /**
   * Creates a new LEDStrip.
   */
  public LEDStrip() {
    canifier.configFactoryDefault(CANIfierConstants.kTimeoutMs); //reset to factory default

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
