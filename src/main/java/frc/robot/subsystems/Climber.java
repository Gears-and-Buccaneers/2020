/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.Constants.ClimberConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private static TalonSRX winchMotor = new TalonSRX(ClimberConstants.kClimberWinch);
  private static TalonSRX hookMotor = new TalonSRX(ClimberConstants.kClimberHook);

  private boolean climberExtended;
  private double startTime;
  
  /**
   * Creates a new Climber.
   */
  public Climber() {
    climberExtended = false;
    startTime = Timer.getFPGATimestamp();

    winchMotor.configFactoryDefault(); //clears all previous settings
    hookMotor.configFactoryDefault();
  }

  /**
   * @return the climberExtended
   */
  public boolean isClimberExtended() {
    return climberExtended;
  }

  /**
   * extends the climber to a set position
   */
  public void extendClimber() {
    while(Timer.getFPGATimestamp() - startTime < ClimberConstants.kExtendTimeInSeconds) {
      winchMotor.set(ControlMode.PercentOutput, -1);
    }

    winchMotor.set(ControlMode.PercentOutput, 0);
    climberExtended = true;
  }

  public void driveOnBar(double left, double right) {
    double leftInput = -left;
    double rightInput =  right;
    double output = leftInput + rightInput;
    hookMotor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
