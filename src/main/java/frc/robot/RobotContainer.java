/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj2.command;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants;
//import frc.robot.commands.ComplexAutoCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

import static edu.wpi.first.wpilibj.XboxController.Button;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  //private final Intake m_intake = new Intake();

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_drivetrain.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_drivetrain
            .arcadeDrive(DriveConstants.kDriveCoefficient * m_driverController.getY(GenericHID.Hand.kLeft),
                         DriveConstants.kTurnCoefficient * m_driverController.getX(GenericHID.Hand.kRight)), m_drivetrain));

    // Add commands to the autonomous command chooser
    //m_chooser.addOption("Simple Auto", m_simpleAuto);
    //m_chooser.addOption("Complex Auto", m_complexAuto);

    //intake default
    //m_intake.setDefaultCommand();

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // // Pop intake out when the 'A' button is pressed.
    // new JoystickButton(m_driverController, Button.kA.value)
    // .whenPressed(new SequentialCommandGroup(
    //     new InstantCommand(m_intake::open, m_intake),
    //     new InstantCommand(m_intake::runNow, m_intake)));
    // //close intake when 'b' is pressed
    // new JoystickButton(m_driverController, Button.kB.value)
    // .whenPressed(new SequentialCommandGroup(
    //   new InstantCommand(m_intake::retract, m_intake),
    //   new InstantCommand(m_intake::stopRunning, m_intake)));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
