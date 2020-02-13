/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.WheelSpinner;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Stage1Spin extends CommandBase {
  private static WheelSpinner m_spinner;
  
  private double startTime;
  /**
   * Creates a new Stage1Spin.
   */
  public Stage1Spin(WheelSpinner spinner) {
    m_spinner = spinner; //superclassing! yay!
    addRequirements(m_spinner); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(Timer.getFPGATimestamp() - startTime < 3){
      m_spinner.run();
    }
    m_spinner.stop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_spinner.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
