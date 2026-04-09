package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootForTime;
import frc.robot.commands.Wait;
import frc.robot.commands.DriveChoreo;
import frc.robot.commands.DrivePath;

public class Autonomous {
    Drivetrain m_drivetrain;
    Shooter m_shoot;
    Intake m_intake;

    double top_speed = -100; // RPM
    double bottom_speed = 5600; // RPM

    double close_top_speed = -700;
    double close_bottom_speed = 4400;

    enum AutoMode {
        CENTER,
        HUMAN,
        DEPOT,
        NEUTRAL_RIGHT,
        NEUTRAL_LEFT,
        TEST
    }

    static SendableChooser<AutoMode> m_autochooser = new SendableChooser<AutoMode>();
    double m_Target = 1;

    // distance for everything is in meters

    public Autonomous(Drivetrain drivetrain, Shooter shoot, Intake intake) {
        m_drivetrain = drivetrain;
        m_shoot = shoot;
        m_intake = intake;

        m_autochooser.addOption("Center Full", AutoMode.CENTER);
        m_autochooser.addOption("Human Pickup", AutoMode.HUMAN);
        m_autochooser.addOption("Depot Pickup", AutoMode.DEPOT);
        m_autochooser.addOption("Neutral Right", AutoMode.NEUTRAL_RIGHT);
        m_autochooser.addOption("Neutral Left", AutoMode.NEUTRAL_LEFT);
        m_autochooser.addOption("Test", AutoMode.TEST);

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
            case CENTER:
                return new SequentialCommandGroup(
                        new DriveChoreo(m_drivetrain, m_shoot, m_intake, "Full_Path", 0.5, true),
                        new ShootForTime(m_drivetrain, m_shoot, m_intake, 7, top_speed, bottom_speed));
            case HUMAN:
                return new SequentialCommandGroup(
                        new DriveChoreo(m_drivetrain, m_shoot, m_intake, "Human", 0.5, true),
                        new ShootForTime(m_drivetrain, m_shoot, m_intake, 7, close_top_speed, close_bottom_speed));
            case DEPOT:
                return new SequentialCommandGroup(
                        new DriveChoreo(m_drivetrain, m_shoot, m_intake, "Depot", 0.5, true),
                        new Wait(m_drivetrain, 0.02),
                        new ShootForTime(m_drivetrain, m_shoot, m_intake, 5, top_speed, bottom_speed));
            case NEUTRAL_RIGHT:
                return new SequentialCommandGroup(
                        new DriveChoreo(m_drivetrain, m_shoot, m_intake, "Neutral_Right_1", 0.5, true),
                        new ShootForTime(m_drivetrain, m_shoot, m_intake, 4.4, close_top_speed, close_bottom_speed),
                        // new Wait(m_drivetrain, 1),
                        new DriveChoreo(m_drivetrain, m_shoot, m_intake, "Neutral_Right_2", 0.5, false),
                        new ShootForTime(m_drivetrain, m_shoot, m_intake, 7, close_top_speed, close_bottom_speed));
            case NEUTRAL_LEFT:
                return new SequentialCommandGroup(
                        new DriveChoreo(m_drivetrain, m_shoot, m_intake, "Neutral_Left_1", 0.5, true),
                        new ShootForTime(m_drivetrain, m_shoot, m_intake, 4.4, close_top_speed, close_bottom_speed),
                        // new Wait(m_drivetrain, 1),
                        new DriveChoreo(m_drivetrain, m_shoot, m_intake, "Neutral_Left_2", 0.5, false),
                        new ShootForTime(m_drivetrain, m_shoot, m_intake, 7, close_top_speed, close_bottom_speed));
            case TEST:
                return new SequentialCommandGroup(
                        new DriveChoreo(m_drivetrain, m_shoot, m_intake, "Test", 0.5, true));
        }
    }

    public void endAuto() {

    }

    public void initAuto() {
        m_drivetrain.resetOdometry();
    }
}
