/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftMaster = 0; //CAN IDs for drivetrain Falcon500
        public static final int kLeftSlave = 1;
        public static final int kRightMaster = 2;
        public static final int kRightSlave = 3;

        public static final boolean kGyroReversed = true;

        public static final int kVoltageCompLevel = 10; //Lowest Voltage Motors should get to in Volts from PDB

        public static final int kEncoderCPR = 2048; //counts per revolution for enocder math
        public static final double kWheelDiameterInches = 6; //wheel diameter TODO: maybe convert to metric?
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR; // TODO: need to include gear ratio math here

        public static final double kTurnCoefficient = 0.7; //Overall Scaling for Turning
        public static final double kDriveCoefficient = 1; //Overall Scaling for Forward/Reverse

        public static final double kRampCoefficient = 0.1; //Time in seconds from 0% to 100%

        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;
      }
    
      public static final class IntakeConstants {
        public static final int kIntakeSolenoidForward = 0; //PCM slot # for solenoid
        public static final int kIntakeSolenoidReverse = 1;
        public static final int kIntakeMotor = 61; //CAN IDs for intake SRX
        public static final int[] kIntakeSolenoidPorts = new int[]{0, 1};
        public static final int runTime = 15;
      }

      public static final class ClimberConstants {
        public static final int kClimberWinch = 61;
        public static final int kClimberHook = 10;
        public static final int kClimberElevator = 7;

        public static final int kExtendTimeInSeconds = 2;

        public static final int[] kClimberSolenoidPorts = new int[]{2, 3};
      }

      public static final class ShooterConstants {
        public static final int kShooterMaster = 20; //CAN IDs for shooter SRXs
        public static final int kShooterSlave = 60;

        //PID Velocity Loop Control constants
        public static final int kSlotIdx = 0; //Says what PID slot is used
        public static final int kPIDLoopIdx = 0; //get the primary PID loop
        public static final int kTimeoutMs = 30; //to make sure config works

        public static final double kP = 0.25; //proportional gain
        public static final double kI = 0.001; //integral gain
        public static final double kD = 20; //derivitive gain
        public static final double kF = 1023.0/7200.0; //FeedForward gain, so 1023 is 100% Talon and 7200 is velocity units from encoder in 100ms
        public static final double Iz = 300; //I-zone
        public static final double PeakOut = 1.00; //Peak motor output

        /**
         * Convert 6000 RPM to units / 100ms.
         * 2048 Units/Rev * 6000 RPM / 600 100ms/min in either direction:
         * velocity setpoint is in units/100ms
         */
        public static final double targetVelocity_UnitsPer100ms = 6000.0 * 2048 / 600;

        //constants for limelight vision
        // These numbers must be tuned for your Robot!  Be careful!
        public static final double STEER_K = 0.03;                    // how hard to turn toward the target
        public static final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
        public static final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        public static final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast
      }

      public static final class BallStorageConstants {
        public static final int kStorageMotorMaster = 30;
        public static final int kStorageMotorSlave = 40;
        
        public static final int entranceSensor = 0;
        public static final int exitSensor = 1;

        public static final double minRecognizeVoltage = 1;
      }

      public static final class CANIfierConstants {
        public static final int kCanifier1 = 50;
        public static final int kCanifier2 = 51;

        public static final int kTimeoutMs = 30;
      }

      public static final class WheelSpinnerConstants {
        public static final int kWheelSpinnerMotor  = 69;

        public static final double wheelSpinnerMotorSpeed = 0.7;

        public static final int[] kSpinnerSolenoidPorts = new int[]{4, 5};
      }
    
      public static final class AutoConstants {
        public static final double kAutoDriveDistanceInches = 60;
        public static final double kAutoBackupDistanceInches = 20;
        public static final double kAutoDriveSpeed = 0.5;

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
      }
    
      public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
      }    
}
