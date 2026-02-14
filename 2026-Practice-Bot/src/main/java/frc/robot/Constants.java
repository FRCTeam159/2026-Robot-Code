// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {

    public enum test {
        CLIMBER, ONEROLLER, TWOROLLERS, ARMGYRO, ARMMOTOR, NONE
    }

    public static test testMode = test.NONE;

    // public static final double kBackRightOffset = -0.169434;
    // public static final double kBackLeftOffset = -0.215332;
    // public static final double kFrontRightOffset = -0.400635;
    // public static final double kFrontLeftOffset = 0.039062;
    // can ids for drive train
    public static final double kBackRightOffset = -0.165771;
    public static final double kBackLeftOffset = -0.214844;
    public static final double kFrontRightOffset = -0.388672;
    public static final double kFrontLeftOffset = 0.050537;

    public static final double limelight_height = 0.58;
    public static final double limelight_angle = -0.8;
    public static final double apriltag_height = 1.1176;

    public static final int kFl_Drive = 3;
    public static final int kFl_Turn = 8;
    public static final int kFl_Encoder = 12;

    public static final int kFr_Drive = 7;
    public static final int kFr_Turn = 6;
    public static final int kFr_Encoder = 10;

    public static final int kBl_Drive = 2;
    public static final int kBl_Turn = 4;
    public static final int kBl_Encoder = 9;

    public static final int kBr_Drive = 5;
    public static final int kBr_Turn = 1;
    public static final int kBr_Encoder = 11;

    public static final int top_shooter = 13;
    public static final int bottom_shooter = 14;
    public static final int shooter_feeder = 15;

    public static final int mTest = 17;
    
    public static final int kPigeonCanId = 30;

    static public final double kDriveGearRatio = 8.14; // MK4i drive (standard)
    static public final double kTurnGearRatio = 21.429; // MK4i turn (all)

    public static final double kWheelRadius = 2;
    public static final double kDistPerRot = (Units.inchesToMeters(kWheelRadius) * 2 * Math.PI) / kDriveGearRatio;
    public static final double kRadiansPerRot = Math.PI * 2 / kTurnGearRatio;
  
    public static final double kRobotLength = Units.inchesToMeters(24); // Waffle side length

    public static final double kFrontWheelBase = Units.inchesToMeters(18.5); // distance bewteen front wheels
    public static final double kSideWheelBase = Units.inchesToMeters(18.5); // distance beteen side wheels
    public static final double kTrackRadius = 0.5
        * Math.sqrt(kFrontWheelBase * kFrontWheelBase + kSideWheelBase * kSideWheelBase);

    public static final double kMaxVelocity = 0.5;
    public static final double kMaxAcceleration = 1;
    public static final double kMaxAngularVelocity = Math.toRadians(720); // radians/s
    public static final double kMaxAngularAcceleration = Math.toRadians(360); // radians/s/s

    //public static final RobotConfig m_EMPTY_CONFIG = new RobotConfig(22.68, 10, );
}
