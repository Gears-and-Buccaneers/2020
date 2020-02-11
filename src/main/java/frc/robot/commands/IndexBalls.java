/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Storage;

import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexBalls extends CommandBase {
  private static Storage m_storage;

  public IndexBalls(Storage subsystem) {
    m_storage = subsystem;
    addRequirements(m_storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_storage.isPresentOnEntry()){
      m_storage.run();
      new WaitCommand(1);
      
    }
    else{
      m_storage.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_storage.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_storage.getNumBalls()>4){
      return true;
    }
    
    return false;
  }
}
