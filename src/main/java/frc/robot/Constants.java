// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants{ //change for final bot
        public static final double ksVolts = 0.60157;
        public static final double ksVoltSecondsPerMeter = 2.4793;
        public static final double kaVoltSecondsSquaredPerMeter = 0.3556;

        public static final double kPDriveVel = 3.3861;

        public static final double kTrackwidthMeters = 0.59; 
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    }

    public static final class AutoConstants{  
        public static final double kMaxSpeedMetersPerSecond = 0.3;
        public static final double kMaxAccelerationMetersPerSecond = 0.3;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class ChasisConstants{
        public static final int frontLeftID = 8;
        public static final int frontRightID = 9;
        public static final int rearLeftID = 6;
        public static final int rearRightID = 1;
    }

}
