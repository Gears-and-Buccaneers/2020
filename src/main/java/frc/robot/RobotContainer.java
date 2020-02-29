/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.WheelSpinner;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Limelight;

import frc.robot.commands.Auto1;
import frc.robot.commands.Auto2;
import frc.robot.commands.IndexBalls;
import frc.robot.commands.ExhaustBalls;
import frc.robot.commands.Stage1Spin;

import static edu.wpi.first.wpilibj.XboxController.Button;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain m_drivetrain = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final Storage m_storage = new Storage();
  private final Climber m_climber = new Climber();
  private final WheelSpinner m_spinner = new WheelSpinner();

  private final Limelight m_limelight = new Limelight();

  //private final LEDStrip m_ledStrip = new LEDStrip();

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

    //Auto1 m_auto1 = new Auto1(m_drivetrain, m_shooter, m_storage);
    //Auto2 m_auto2 = new Auto2(m_drivetrain);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_drivetrain.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(() -> m_drivetrain
            .arcadeDrive(DriveConstants.kDriveCoefficient * m_driverController.getRawAxis(1),
                         DriveConstants.kTurnCoefficient * m_driverController.getRawAxis(4)), m_drivetrain));

    //make the bumpers control the bar side to side motors.
    // m_climber.setDefaultCommand(
    //   new RunCommand(
    //         () -> m_climber
    //     .driveOnBar(m_driverController.getRawAxis(3), m_driverController.getRawAxis(4))
    // ));


    // m_limelight.setDefaultCommand(
    //   new RunCommand(() -> m_limelight.update(true)) //makes the limelight update to the smartdashboard constantly
    // );

    m_storage.setDefaultCommand(
      new IndexBalls(m_storage)
      //new RunCommand(m_storage::stop, m_storage)
    );

    m_shooter.setDefaultCommand(
      new RunCommand(m_shooter::stopShooter, m_shooter)
    );

    m_spinner.setDefaultCommand(
      new RunCommand(m_spinner::getColor, m_spinner)
    );
    // m_intake.setDefaultCommand(
    //   //new RunCommand(m_intake.runWithAnalog(m_driverController.getTriggerAxis(Hand.kRight))));
    //   new SequentialCommandGroup(
    //     //new InstantCommand(m_intake::retract, m_intake),
    //     new InstantCommand(m_intake::stopRunning, m_intake)
    //     ));
    //   //new RunCommand(
        
    //   //)s
    
    // m_ledStrip.setDefaultCommand(
    //   new RunCommand(m_ledStrip::setColor, m_ledStrip)
    // );


    // Add commands to the autonomous command chooser
    //m_chooser.addOption("simple auto", m_auto1);
    //m_chooser.addOption("auto using ramsetecommand", m_auto2);

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
    // Pop intake out when the right bumper is pressed.
    new JoystickButton(m_driverController, Button.kBumperRight.value)
      .whenPressed( new SequentialCommandGroup(
          new InstantCommand(m_intake::open, m_intake),
          new InstantCommand(m_intake::runNow, m_intake),
          new IndexBalls(m_storage)
      )
    );

    // bring intake in when button is released
    new JoystickButton(m_driverController, Button.kBumperLeft.value).whenPressed(
      new InstantCommand(m_intake::stopRunning, m_intake).andThen(
      new InstantCommand(m_intake::retract, m_intake),
      new InstantCommand(m_storage::stop, m_storage))
    );

    //shoot balls while the x is held
    new JoystickButton(m_driverController, Button.kX.value).whileHeld(
          new InstantCommand(m_shooter::runOpenLoop, m_shooter)
          //new ExhaustBalls(m_storage, m_shooter)
    );

    //push balls away while the left stick is pressed
    new JoystickButton(m_driverController, Button.kStickLeft.value)
      .whenPressed(new SequentialCommandGroup(
          new InstantCommand(m_intake::open, m_intake),
          new InstantCommand(m_intake::reverse, m_intake),
          new InstantCommand(m_storage::reverse, m_storage)
    ));

    //play music while back is held :)
    new JoystickButton(m_driverController, Button.kBack.value).whileHeld(new RunCommand(m_drivetrain::playMusic, m_drivetrain));

    //open wheel spinner and run while 'B' is HELD
    new JoystickButton(m_driverController, Button.kB.value).whileHeld(
        new RunCommand(m_spinner::extend, m_spinner).withTimeout(3).andThen(new Stage1Spin(m_spinner))
    );

    //extend climber when start is pressed
    new JoystickButton(m_driverController, Button.kStart.value).whileHeld(
      new InstantCommand(m_climber::extendClimber, m_climber));

    //upper dpad button for transport testing. runs transport when pressed
    new POVButton(m_driverController, 0).whenPressed(
      new RunCommand(m_storage::run, m_storage)
    );

    //stops transport when released
    new POVButton(m_driverController, 0).whenReleased(
      new InstantCommand(m_storage::stop, m_storage)
    );
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
    //return new Auto1(m_drivetrain);
  }

}
