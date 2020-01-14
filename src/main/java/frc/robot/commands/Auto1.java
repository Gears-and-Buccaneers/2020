/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class Auto1 extends SequentialCommandGroup {
  /**
   * Creates a new Auto1.
   */
  public Auto1(Drivetrain driveSubsystem) {
    addCommands(
        // Drive backwards off of the initiation line and shoot the balls
        new StartEndCommand(
            // Start driving forward at the start of the command
            () -> driveSubsystem.arcadeDrive(-AutoConstants.kAutoDriveSpeed, 0),
            // Stop driving at the end of the command
            () -> driveSubsystem.arcadeDrive(0, 0), driveSubsystem)
            // Reset the encoders before starting
            .beforeStarting(driveSubsystem::resetEncoders)
            // End the command when the robot's driven distance exceeds the desired value
            .withInterrupt(() -> driveSubsystem.getAverageEncoderDistance()
                >= AutoConstants.kAutoDriveDistanceInches));
  }
}
