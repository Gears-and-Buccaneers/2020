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
        public static final int kLeftMaster = 0;
        public static final int kLeftSlave = 1;
        public static final int kRightMaster = 2;
        public static final int kRightSlave = 3;

        public static final int kVoltageCompLevel = 11;
        
        public static final int kEncoderCPR = 4096;
        public static final double kWheelDiameterInches = 6;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;
      }
    
      public static final class IntakeConstants {
        public static final int kIntakeSolenoidForward = 0;
        public static final int kIntakeSolenoidReverse = 1;
        public static final int kIntakeMotor = 10;
        public static final int[] kIntakeSolenoidPorts = new int[]{0, 1};
        public static final int runTime = 15;
      }

      public static final class ShooterConstants {
        public static final int kShooterMaster = 20;
        public static final int kShooterSlave = 21;
      }

      public static final class BallStorageConstants {
        public static final int kStorageMotorMaster = 30;
        public static final int kStorageMotorSlave = 31;   
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
