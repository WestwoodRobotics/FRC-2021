// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants{
        public static final int P_DRIVE_LEFT_MASTER = 0,
                                P_DRIVE_LEFT_FOLLOW = 1,
                                P_DRIVE_RIGHT_MASTER = 3,
                                P_DRIVE_RIGHT_FOLLOW = 2;

        // Feedforward for FTC Mats
        // public static final double C_kS = 1.44,
        //                            C_kV = 3.26,
        //                            C_kA = 0.693;
        
        // Feedforward for Tile
        public static final double C_kS = 1.12,
                                   C_kV = 3.08,
                                   C_kA = 0.622;

        // Left PID                               
        public static final double C_kP_LEFT = 5,//8,
                                   C_kI_LEFT = 0,
                                   C_kD_LEFT = 0;
        
        // Right PID
        public static final double C_kP_RIGHT = 5,//20,
                                   C_kI_RIGHT = 0,
                                   C_kD_RIGHT = 0;

        public static final double C_TRACK_WIDTH_METERS = 0.5588;
        public static final double C_DRIVE_EPR = 8192;
        public static final double C_WHEEL_DIAMETER_METERS = 0.1524;

        public static final double C_MAX_VOLTAGE = 10.0;

        public static final double C_kB_RAMSETE = 2.0,
                                   C_kZ_RAMSETE = 0.7;

        public static double ticksToMeters(double ticks){
            return ticks*Math.PI*C_WHEEL_DIAMETER_METERS/C_DRIVE_EPR;
        }

        public static double metersToTicks(double meters){
            return meters*C_DRIVE_EPR/(Math.PI*C_WHEEL_DIAMETER_METERS);
        }
    
        public static double radiansToMeters(double radians){
            return C_TRACK_WIDTH_METERS/2 * radians;
        }
    
        public static double metersToRadians(double meters){
            return meters * 2/C_TRACK_WIDTH_METERS;
        }
    }
    
    public static final class IntakeConstants{
        public static final int P_INTAKE_talSRX_1 = 0;
        
        public static final int P_INTAKE_sol_1 = 0;
        //public static final int P_INTAKE_sol_2 = 0;

        public static final double C_INTAKE_SPEED = 1.0;
    }

    public static final class ShooterConstants{// TODO Set motor ports
        public static final int P_SHOOTER_spMAX_1 = 0,
                                P_SHOOTER_spMAX_2 = 0,
                                P_ACTUATOR = 0;

        public static final double C_kS = 0,
                                   C_kV = 0,
                                   C_kA = 0,
                                   C_kP = 0,
                                   C_kI = 0,
                                   C_kD = 0;
        
        public static enum E_SHOOTER_POS{
            CLOSE,
            FAR;
        }
        public static final double  C_SHOOTER_SPEED_CLOSE = 0,
                                    C_SHOOTER_SPEED_FAR = 0,
                                    C_SHOOTER_SPEED_TOLERANCE = 0,

                                    C_ACTUATOR_MIN_PWM_MS = 1050, // About 1 second, the minimum PMW value
                                    C_ACTUATOR_MAX_PWM_MS = 1950, // About 2 second, the maximum PMW value

                                    C_ACTUATOR_MIN_DEG = 0, // Do the math nerd
                                    C_ACTUATOR_MAX_DEG = 0;
        
    }

    public static final class MagazineConstants{
        public static final int //P_MAGAZINE_vicSPX_1 = 8,
                                P_MAGAZINE_talSRX_2 = 5;
    }

}
