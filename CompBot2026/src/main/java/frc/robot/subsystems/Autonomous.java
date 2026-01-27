package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.DrivePath;

public class Autonomous {
    Drivetrain m_drivetrain;
    TagDetector m_Detector;
    public static final int CENTER_AUTO = 1;
    public static final int SIDE_AUTO = 2;
    public static final int DRIVE_TO_TAG = 3;
    public static final int DRIVE_PATH = 4;
    public static final int DRIVE_STRAIGHT = 5;
    static SendableChooser<Integer> m_autochooser = new SendableChooser<Integer>();
    double m_sideTarget = 2.95;
    double m_centerTarget = 1.25;
    boolean m_center = false;

    public Autonomous(Drivetrain drivetrain, TagDetector Detector) {
        m_drivetrain = drivetrain;
        m_Detector = Detector;
        m_autochooser.setDefaultOption("Center Auto",CENTER_AUTO);
        m_autochooser.addOption("Side Auto", SIDE_AUTO);
        m_autochooser.addOption("Drive To Tag", DRIVE_TO_TAG);
        m_autochooser.addOption("Drive Path", DRIVE_PATH);
        m_autochooser.addOption("Drive Straight", DRIVE_STRAIGHT);
        SmartDashboard.putData(m_autochooser);

        SmartDashboard.putBoolean("Center", m_center);
        // if (m_center)
        //     m_Target = 1.35;
        // else
        //     m_Target = 3.1;
                // SmartDashboard.putNumber("target", m_driveStraitTarget);
    }

    public SequentialCommandGroup getCommand() {
        // m_driveStraitTarget = SmartDashboard.getNumber("target", 0);
        DrivePath.setEndAtTag(false);
        int automode = m_autochooser.getSelected();
        switch (automode) {
            default:
                return null;
            case DRIVE_TO_TAG:
                return new SequentialCommandGroup(new DriveToTag(m_drivetrain));
            case DRIVE_PATH:
                return new SequentialCommandGroup(new DrivePath(m_drivetrain, m_sideTarget));
        }

        // driveToTag();
        // return new SequentialCommandGroup(new DriveToTag(m_drivetrain));
    }

    public void endAuto() {
        TagDetector.setTargeting(false);
    }

    public void initAuto() {
        SmartDashboard.getBoolean("Center", m_center);
        TagDetector.setTargeting(true);
    }
}
