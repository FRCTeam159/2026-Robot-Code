package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.ShootForTime;
import frc.robot.commands.Wait;
import frc.robot.commands.DriveChoreo;
import frc.robot.commands.DrivePath;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.ResetWheels;

public class Autonomous {
    Drivetrain m_drivetrain;
    //Shooter m_shoot;

    enum AutoMode {
        DRIVE_TO_TAG,
        DRIVE_PATH,
        DRIVE_STRAIGHT,
        SHOOT,
        ALIGN,
        CHOREO
    }

    static SendableChooser<AutoMode> m_autochooser = new SendableChooser<AutoMode>();
    double m_Target = 1;
    // distance for everything is in meters

    public Autonomous(Drivetrain drivetrain/*, Shooter shoot*/) {
        m_drivetrain = drivetrain;
        //m_shoot = shoot;

        m_autochooser.addOption("Drive To Tag", AutoMode.DRIVE_TO_TAG);
        m_autochooser.addOption("Drive Path", AutoMode.DRIVE_PATH);
        m_autochooser.addOption("Drive Straight", AutoMode.DRIVE_STRAIGHT);
        m_autochooser.addOption("Simple Shoot", AutoMode.SHOOT);
        m_autochooser.addOption("Align With Tag", AutoMode.ALIGN);
        m_autochooser.addOption("Choreo", AutoMode.CHOREO);

        SmartDashboard.putData(m_autochooser);
    }

    public SequentialCommandGroup getCommand() {
        // m_driveStraitTarget = SmartDashboard.getNumber("target", 0);
        DrivePath.setEndAtTag(false);
        AutoMode automode = m_autochooser.getSelected();
        return getAutoSequence(automode);
        // driveToTag();
        // return new SequentialCommandGroup(new DriveToTag(m_drivetrain));
    }

    private SequentialCommandGroup getAutoSequence(AutoMode automode) {
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
            case CHOREO:
                return new SequentialCommandGroup(
                    new ResetWheels(m_drivetrain),
                    new DriveChoreo(m_drivetrain, "straight_path")
                );
        }
    }

    public void endAuto() {
        
    }

    public void initAuto() {
        m_drivetrain.resetOdometry();
    }
}
