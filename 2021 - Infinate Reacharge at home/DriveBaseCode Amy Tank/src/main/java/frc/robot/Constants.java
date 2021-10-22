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
    //public static final int Joystick_ID_L = 0;
    //public static final int Joystick_ID_R = 4;
    
    public static final class DriveConstants {
        public static final int FR_ID = 4;
        public static final int FL_ID = 33;
        public static final int BR_ID = 34;
        public static final int BL_ID = 18;
    }

    public final class OIConstraints {
        public static final int Xbox_ID = 0;
    }
   
}
