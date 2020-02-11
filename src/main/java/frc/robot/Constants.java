/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftMaster = 0; //CAN IDs for drivetrain Falcon500
        public static final int kLeftSlave = 1;
        public static final int kRightMaster = 2;
        public static final int kRightSlave = 3;

        public static final int kVoltageCompLevel = 11; //Lowest Voltage Motors should get to in Volts from PDB

        public static final int kEncoderCPR = 2048; //counts per revolution for enocder math
        public static final double kWheelDiameterInches = 6; //wheel diameter TODO: maybe convert to metric?
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR; // TODO: need to include gear ratio math here

        public static final double kTurnCoefficient = 1; //Overall Scaling for Turning
        public static final double kDriveCoefficient = 1; //Overall Scaling for Forward/Reverse

        public static final double kRampCoefficient = 1; //Time in seconds from 0% to 100%
      }
    
      public static final class IntakeConstants {
        public static final int kIntakeSolenoidForward = 0; //PCM slot # for solenoid
        public static final int kIntakeSolenoidReverse = 1;
        public static final int kIntakeMotor = 10; //CAN IDs for intake SRX
        public static final int[] kIntakeSolenoidPorts = new int[]{0, 1};
        public static final int runTime = 15;
      }

      public static final class ClimberConstants {
        public static final int kClimberWinch = 40;
        public static final int kClimberHook = 41;
        public static final int kClimberElevator = 42;
      }

      public static final class ShooterConstants {
        public static final int kShooterMaster = 20; //CAN IDs for shooter SRXs
        public static final int kShooterSlave = 21;

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
      }

      public static final class BallStorageConstants {
        public static final int kStorageMotorMaster = 30;
        public static final int kStorageMotorSlave = 31;
        
        public static final int entranceSensor = 0;
        public static final int exitSensor = 1;

        public static final double minRecognizeVoltage = 5;
      }

      public static final class CANIfierConstants {
        public static final int kCanifier1 = 50;
        public static final int kCanifier2 = 51;

        public static final int kTimeoutMs = 30;
      }
    
      public static final class AutoConstants {
        public static final double kAutoDriveDistanceInches = 60;
        public static final double kAutoBackupDistanceInches = 20;
        public static final double kAutoDriveSpeed = 0.5;
      }
    
      public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
      }    
}
