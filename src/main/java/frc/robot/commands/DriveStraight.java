/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveStraight extends CommandBase {
  private Drivetrain m_drivetrain;
  private double distance;
  private double timeinit;
  
  /**
   * Creates a new DriveStraight.
   */
  public DriveStraight(Drivetrain drivetrain, double distanceInput) {
    m_drivetrain = drivetrain;
    distance = distanceInput;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeinit = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(Timer.getFPGATimestamp()-timeinit < 2){
      m_drivetrain.drive(-0.5, -0.58);
    }
    m_drivetrain.drive(0, 0);
    System.out.println("finished drive auton");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
    System.out.println("finished drive auton in exec");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
