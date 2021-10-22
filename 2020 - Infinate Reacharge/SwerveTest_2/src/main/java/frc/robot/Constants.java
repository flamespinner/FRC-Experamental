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
    public class DriveConstants {
        public static final int DRIVE_FR = 1;
        public static final int DRIVE_FL = 2;
        public static final int DRIVE_BR = 3;
        public static final int DRIVE_BL = 4;
        public static final int TURNING_FR = 5;
        public static final int TURNING_FL = 6;
        public static final int TURNING_BR = 7;
        public static final int TURNING_BL = 8;
        
        public static final boolean DRIVE_FR_ENC_INVERTED = false;
        public static final boolean DRIVE_FL_ENC_INVERTED = false;
        public static final boolean DRIVE_BR_ENC_INVERTED = false;
        public static final boolean DRIVE_BL_ENC_INVERTED = false;
        public static final boolean TURNING_FR_ENC_INVERTED = false;
        public static final boolean TURNING_FL_ENC_INVERTED = false;
        public static final boolean TURNING_BR_ENC_INVERTED = false;
        public static final boolean TURNING_BL_ENC_INVERTED = false;

        // Locations for the swerve drive modules relative to the robot center.
        
        public static final double FR_MODULE_POS_X = 0.381;
        public static final double FR_MODULE_POS_Y = -0.381;
        public static final double BR_MODULE_POS_X = -0.381;
        public static final double BR_MODULE_POS_Y = -0.381;
        public static final double FL_MODULE_POS_X = 0.381; 
        public static final double FL_MODULE_POS_Y = 0.381;
        public static final double BL_MODULE_POS_X = -0.381;
        public static final double BL_MODULE_POS_Y = 0.381;

        //TODO: these numbers might be very wrong (max is 20,000rpm)
        public static final double MAX_SPEED_MperSEC = 32;
        public static final double MAX_SPEED_RADperSEC = 2093;
    }
    public class ModuleConstants {
        public static final double DRIVE_CONTROLLER_P = 1;
        public static final double TURNING_CONTROLLER_P = 1;

        public static final double MAX_ANGULAR_SPEED_RADperSEC = 2 * Math.PI;
        public static final double MAX_ANGULAR_ACCLERATION_RADperSECsquared = 2 * Math.PI;

        public static final double TICperIn = 4096/4;
        // 4069 counts per rev, wheels with 4in circumfrence
        
    }
    public class OIConstants {
        public static final int JOYSTICK_PORT_ID = 0;
    }
}
