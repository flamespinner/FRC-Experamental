/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 * 
 * global constants are declared and initailized here
 */
public final class Constants {

    /**
     * global constants used in manipulating the drive train
     * (motor controller ids, conversion rate from ticks to feet for encoders, etc.)
     */
    public final class DriveConstants {
        public static final int FALCON_FR_ID = 44;
        public static final int FALCON_BR_ID = 41;
        public static final int FALCON_FL_ID = 43;
        public static final int FALCON_BL_ID = 42;

        public static final double TIC_FT = ((Math.PI)/2014)/10.75;

        public static final double DRIVE_P = 4;
        public static final double DRIVE_I = 1;

        public static final int timeout_ms = 0;
    }

    public final class AutoConstants {
        public static final double ksVolts = 0.662;
        public static final double kvVoltsSecondsPerMeter = 1.57;
        public static final double ka = 0.143;
        public static final double kaVoltSecondsSquaredPerMeter = 0.999;

        public static final double kPDriveVel = 1.35;
        public static final double kPDriveVel0 = 0;
        //Values to try: kP 2.81, 0.0151 
        public static final double kTrackwidthMeters = 1.736225356863555;// Use calculated from FRC Tool
        
        public static final double kMaxSpeedMetersPerSecond = 3; //3
        public static final double kMaxAccelerationMetersPerSecondSquared =3; //3

        //baseline values for Ramsete  follower in units of meters and seconds
        public static final double kRamseteB = 2; //This should be good for most robots
        public static final double kRamseteZeta = 0.7; //This should be good for most robots

        public static final double gearRatio = 8.45; //This shouldn't change
        public static final double CPR = 2048;
    }

    public static class VelocityConversions {
        public static final double WheelCircumference = (2 * Math.PI * 3);
        public static final double SensorUnitsPerRotation = 2048;
        public static final double DriveBaseDeduction = 8.75;
        public static final double InchesToMeter = (39.37);
        public static final double SensorToMeters = (1.0 / SensorUnitsPerRotation * WheelCircumference * (1.0 / InchesToMeter) /2.0);
        //public static final double WheelDiameterMeters = ();
    }

    /**
     * global constants used in the queue (intake, indexer, conveyor)
     * (motor controller ids, speeds, etc.) 
     */
    public final class QueueConstants {
        public static final int INTAKE_RED_ID = 2;
        public static final int INDEXER_RED_ID_1 = 16;
        public static final int INDEXER_RED_ID_2 = 31;
        public static final int INDEXER_BOTTOM_RED_ID = 20;
        public static final int CONVEYOR_RED_ID_1 = 32;

        public static final int LINEBREAK_TRANSMITTER_ID = 0;
        public static final int LINEBREAK_RECIVER_ID = 1;
       
        public static final double INTAKE_SPEED = .5;
        public static final double INDEXER_SPEED = .75;
        public static final double INDEXER_BOTTOM_SPEED = .75;
        public static final double CONVEYOR_SPEED = .25;
    }

    /**
     * global constants used in the shooter
     * (motor controller ids, pid values/ motor speeds, etc.) 
     */
    public final class ShooterConstants {
        public static final int SHOOTER_SPARK_ID = 53; 
        public static final int LIMELIGHT_SERVO_ID = 3;

        public static final double HIGH_GOAL_SPEED = -.80; //Changed
        public static final int LIMELIGHT_ANGLE_SETPOINT = 57 + 90; //far shoot
        public static final int LIMELIGHT_ANGLE_SETPOINT2 = 10 + 90; //key shoot

        public static final double SHOOTER_P = 1;
        public static final double SHOOTER_I = 1;
        public static final double SHOOTER_SETPOINT = 4400; 
        public static final double SHOOTER_MAX_VELOCITY = 5676;

        public static final double AUTO_SHOOT_TIME = 5;
            //2992.13 rpm = 7.6 m/s
            // max is 5676?
    }

    /**
     * global constants used in the climber
     * (motor controller ids, pid values/ motor speeds, etc.) 
     */
    public final class ClimberConstants {
        public static final int CLIMB_RED_ID = 25;
        public static final int WINCH_NEO_ID = 54;
    
        public static final double WINCH_SPEED = -.5;
        public static final double CLIMB_SPEED = .5;
    }

    /**
     * global constants used in limelight
     * (idk)
     */
    public final class LimelightConstants {

    }
    
    /**
     * global constants used in user input/ output 
     * (xbox controller port id, etc.)
     */
    public final class OIConstants {
        public static final int XBOX_ID = 0;
        public static final int HELMS_ID = 1;
        //public static final int buttonBoard = 3; //TODO BUTTONBOARDTEST
    }
}