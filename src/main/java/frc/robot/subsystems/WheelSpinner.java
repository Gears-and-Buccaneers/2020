/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants.WheelSpinnerConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WheelSpinner extends SubsystemBase {
  private static TalonSRX wheelMotor = new TalonSRX(WheelSpinnerConstants.kWheelSpinnerMotor); //motor

  private static DoubleSolenoid spinnerSol = new DoubleSolenoid(WheelSpinnerConstants.kSpinnerSolenoidPorts[0], WheelSpinnerConstants.kSpinnerSolenoidPorts[1]); //makes a new doublesolenoid for the extend and retract.

  private final I2C.Port i2cPort = I2C.Port.kOnboard; //i2c port for color sensor

  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort); //color sensor
  private final ColorMatch colorMatcher = new ColorMatch(); //to store if color has been matched

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429); //colors we are looking for
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  private Color detectedColor; //the color we are actually detecting from the sensor
  private String colorString;
  private ColorMatchResult match; //the match result

  
  /**
   * Creates a new WheelSpinner.
   */
  public WheelSpinner() {
    colorMatcher.addColorMatch(kBlueTarget); //add targets to the matcher
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);    

    wheelMotor.configFactoryDefault(); //reset any previous configs on wheel motor.

    wheelMotor.setNeutralMode(NeutralMode.Brake);

    spinnerSol.set(Value.kForward);
  }

  public String getColor() {
    detectedColor = colorSensor.getColor(); //get data from sensor
    match = colorMatcher.matchClosestColor(detectedColor); //attempt to match

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
    
    SmartDashboard.putString("Detected Color", colorString);
    return colorString;
  }

  public void run() {
    wheelMotor.set(ControlMode.PercentOutput, WheelSpinnerConstants.wheelSpinnerMotorSpeed); //refer to constants for set wheel speed
  }

  public void stop() {
    wheelMotor.set(ControlMode.PercentOutput, 0);
  }

  public void extend() {
    spinnerSol.set(Value.kReverse); //extends the assembly
  }

  public void retract() {
    spinnerSol.set(Value.kForward); //retracts the assembly
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
