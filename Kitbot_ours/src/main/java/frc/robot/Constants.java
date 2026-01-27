// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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


    public static final int Left_lead_id = 1;
    public static final int left_follow_id = 2;
    public static final int right_lead_id = 3;
    public static final int right_follow_id = 4;

    public static final int intake_id = 6;
    public static final int shoot_id = 5;
}
