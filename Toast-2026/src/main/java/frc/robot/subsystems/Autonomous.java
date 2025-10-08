package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.Eject;
import frc.robot.commands.GoToShelf;
import frc.robot.commands.DrivePath;
import frc.robot.commands.DriveStraight;

public class Autonomous {
    Drivetrain m_drivetrain;
    TagDetector m_Detector;
    Arm m_Arm;
    public static final int DRIVE_STRAIGHT = 1; 
    public static final int DRIVE_TO_TAG = 2;
    public static final int AUTO_TEST = 3;
    public static final int DRIVE_PATH = 4;
    static SendableChooser<Integer> m_autochooser = new SendableChooser<Integer>();
    double m_driveStraitTarget = 1.5;

    public Autonomous(Drivetrain drivetrain, TagDetector Detector, Arm arm) {
        m_drivetrain = drivetrain;
        m_Detector = Detector;
        m_Arm = arm;
        m_autochooser.setDefaultOption("Drive Straight", DRIVE_STRAIGHT);
        m_autochooser.addOption("Drive To Tag", DRIVE_TO_TAG);
        m_autochooser.addOption("Drive Path", DRIVE_PATH);
        m_autochooser.addOption("Auto Test", AUTO_TEST);
        SmartDashboard.putData(m_autochooser);
        //SmartDashboard.putNumber("target", m_driveStraitTarget);
    }

    public SequentialCommandGroup getCommand() {
        //m_driveStraitTarget = SmartDashboard.getNumber("target", 0);
        DriveStraight.setEndAtTag(false);
        DrivePath.setEndAtTag(false);
        int automode = m_autochooser.getSelected();
        switch (automode) {
            default:
            return null;
            case DRIVE_STRAIGHT:
                 return new SequentialCommandGroup(new DriveStraight(m_drivetrain, m_driveStraitTarget));
            case DRIVE_TO_TAG:
                return new SequentialCommandGroup(new DriveToTag(m_drivetrain));
            case DRIVE_PATH:
                return new SequentialCommandGroup(new DrivePath(m_drivetrain, m_driveStraitTarget));
            case AUTO_TEST:
                DriveStraight.setEndAtTag(true);
                return new SequentialCommandGroup(
                    new GoToShelf(m_Arm),
                    new DrivePath(m_drivetrain, m_driveStraitTarget),
                    new DriveToTag(m_drivetrain),
                    new Eject(m_Arm)
                );
        }
        
        //driveToTag();
        //return new SequentialCommandGroup(new DriveToTag(m_drivetrain));
    }

    public void endAuto() {
        TagDetector.setTargeting(false);
    }

    public void initAuto() {
        TagDetector.setTargeting(true);
    }
}
