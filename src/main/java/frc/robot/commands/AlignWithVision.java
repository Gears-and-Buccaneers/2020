/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AlignWithVision extends CommandBase {
  private static Drivetrain m_drivetrain = new Drivetrain();
  private static Limelight m_limelight = new Limelight();

  // Constants: tune driving and steering control constants
  private double m_steeringKP = 0.055;
  private double m_targetArea = 2.1;
  private double m_driveKP = 0.80;

  /**
   * Creates a new AlignWithVision.
   */
  public AlignWithVision(Drivetrain drivetrain, Limelight limelight) {
    m_drivetrain = drivetrain;
    m_limelight = limelight;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain, m_limelight);

    SmartDashboard.putNumber("Steering KP", 0.055);  
    SmartDashboard.putNumber("min TA", 2.1);
    SmartDashboard.putNumber("Driving KP", 0.80);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_steeringKP = SmartDashboard.getNumber("Steering KP", 0.0);
    m_targetArea = SmartDashboard.getNumber("min TA", 0.0);
    m_driveKP = SmartDashboard.getNumber("Driving KP", 0.0);

    SmartDashboard.putNumber("Left", 10000);
    SmartDashboard.putNumber("Right", 10000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double right = m_limelight.getTX()*m_steeringKP; // Right Y
    double left  = (m_targetArea-m_limelight.getTA())*m_driveKP; // Left X
    SmartDashboard.putNumber("target area", m_limelight.getTA());

    // m_DriveTrain.teleop_drive(left, right); // Drive until the target is at desired distance
    SmartDashboard.putNumber("Left", left);
    SmartDashboard.putNumber("Right", right);
    if (m_limelight.isTargetAvalible()) {
      if (m_limelight.getTA() >= m_targetArea) {
        left = 0;
        right = 0;
      }
    } else {
     left = 0;
     right = 0;
    }

    m_drivetrain.arcadeDrive(left, right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
