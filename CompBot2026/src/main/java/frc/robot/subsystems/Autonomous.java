package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.DrivePath;

public class Autonomous {
    Drivetrain m_drivetrain;
    public static final int DRIVE_TO_TAG = 1;
    public static final int DRIVE_PATH = 2;
    static SendableChooser<Integer> m_autochooser = new SendableChooser<Integer>();
    double m_Target = 1;
    // distance for everything is in meters

    public Autonomous(Drivetrain drivetrain) {
        m_drivetrain = drivetrain;
        m_autochooser.addOption("Drive To Tag", DRIVE_TO_TAG);
        m_autochooser.addOption("Drive Path", DRIVE_PATH);
        SmartDashboard.putData(m_autochooser);
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
                return new SequentialCommandGroup(new DrivePath(m_drivetrain, m_Target));
        }

        // driveToTag();
        // return new SequentialCommandGroup(new DriveToTag(m_drivetrain));
    }

    public void endAuto() {
        
    }

    public void initAuto() {
        
    }
}
