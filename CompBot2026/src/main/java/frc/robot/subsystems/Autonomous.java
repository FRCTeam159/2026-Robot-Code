package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.ShootForTime;
import frc.robot.commands.Wait;
import frc.robot.commands.DrivePath;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.ResetWheels;

public class Autonomous {
    Drivetrain m_drivetrain;
    //Shooter m_shoot;

    public static final int DRIVE_TO_TAG = 1;
    public static final int DRIVE_PATH = 2;
    public static final int DRIVE_STRAIGHT = 3;
    public static final int SHOOT = 4;
    public static final int ALIGN = 5;
    public static final int PATH_PLANNER = 6;

    static SendableChooser<Integer> m_autochooser = new SendableChooser<Integer>();
    double m_Target = 1;
    // distance for everything is in meters

    public Autonomous(Drivetrain drivetrain/*, Shooter shoot*/) {
        m_drivetrain = drivetrain;
        //m_shoot = shoot;

        m_autochooser.addOption("Drive To Tag", DRIVE_TO_TAG);
        m_autochooser.addOption("Drive Path", DRIVE_PATH);
        m_autochooser.addOption("Drive Straight", DRIVE_STRAIGHT);
        m_autochooser.addOption("Simple Shoot", SHOOT);
        m_autochooser.addOption("Align With Tag", ALIGN);
        m_autochooser.addOption("Path Planner", PATH_PLANNER);

        SmartDashboard.putData(m_autochooser);
    }

    public SequentialCommandGroup getCommand() {
        // m_driveStraitTarget = SmartDashboard.getNumber("target", 0);
        DrivePath.setEndAtTag(false);
        int automode = m_autochooser.getSelected();
        return getAutoSequence(automode);
        // driveToTag();
        // return new SequentialCommandGroup(new DriveToTag(m_drivetrain));
    }

    private SequentialCommandGroup getAutoSequence(int automode) {
     switch (automode) {
            default:
                return null;
            case DRIVE_TO_TAG:
                return new SequentialCommandGroup(new DriveToTag(m_drivetrain));
            case DRIVE_PATH:
                return new SequentialCommandGroup(
                    new ResetWheels(m_drivetrain),
                    new DrivePath(m_drivetrain, m_Target)
                    );
            case DRIVE_STRAIGHT:
                return new SequentialCommandGroup(
                    new ResetWheels(m_drivetrain),
                    new DriveStraight(m_drivetrain, -1)
                    );
            case SHOOT:
                return new SequentialCommandGroup(
                    new ResetWheels(m_drivetrain),
                    new DriveStraight(m_drivetrain, -1),
                    new ShootForTime()
                );
            case ALIGN:
                return new SequentialCommandGroup(
                    new ResetWheels(m_drivetrain),
                    new DriveStraight(m_drivetrain, -3),
                    new Wait(m_drivetrain, 1.5),
                    new DriveToTag(m_drivetrain)
                );
            case PATH_PLANNER:
        }

    }

    public void endAuto() {
        
    }

    public void initAuto() {
        m_drivetrain.resetOdometry();
    }
}
