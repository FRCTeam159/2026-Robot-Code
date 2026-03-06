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
    Shooter m_shoot;
    Intake m_intake;

    enum AutoMode {
        SHOOT,
        ALIGN,
        HUMAN,
        DEPOT,
        CENTER,
        WIGGLE
    }

    static SendableChooser<AutoMode> m_autochooser = new SendableChooser<AutoMode>();
    double m_Target = 1;
    // distance for everything is in meters

    public Autonomous(Drivetrain drivetrain, Shooter shoot, Intake intake) {
        m_drivetrain = drivetrain;
        m_shoot = shoot;
        m_intake = intake;

        m_autochooser.addOption("Simple Shoot", AutoMode.SHOOT);
        m_autochooser.addOption("Align With Tag", AutoMode.ALIGN);
        m_autochooser.addOption("Human Pickup", AutoMode.HUMAN);
        m_autochooser.addOption("Depot Pickup", AutoMode.DEPOT);
        
        m_autochooser.addOption("Test Wiggle", AutoMode.WIGGLE);

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
        case SHOOT:
            return new SequentialCommandGroup(
                new ResetWheels(m_drivetrain),
                new DriveStraight(m_drivetrain, -1)
            );
        case ALIGN:
            return new SequentialCommandGroup(
                new ResetWheels(m_drivetrain),
                new DriveStraight(m_drivetrain, -3),
                new Wait(m_drivetrain, 0.5),
                new DriveToTag(m_drivetrain)
            );
        case HUMAN:
            return new SequentialCommandGroup(
                new ResetWheels(m_drivetrain),
                new DriveChoreo(m_drivetrain, m_shoot, m_intake, "Wiggle", 1),
                new Wait(m_drivetrain, 0.02),
                new DriveChoreo(m_drivetrain, m_shoot, m_intake, "Comp_1_1", 0.5),
                new ShootForTime(m_drivetrain, m_shoot, 100)
            );
        case DEPOT:
            return new SequentialCommandGroup(
                new ResetWheels(m_drivetrain),
                new DriveChoreo(m_drivetrain, m_shoot, m_intake, "Wiggle", 1),
                new Wait(m_drivetrain, 0.02),
                new DriveChoreo(m_drivetrain, m_shoot, m_intake, "Comp_2_1", 0.5)
            );
        case CENTER:
            return new SequentialCommandGroup(
                new ResetWheels(m_drivetrain),
                new DriveChoreo(m_drivetrain, m_shoot, m_intake, "Wiggle", 1),
                new Wait(m_drivetrain, 0.02),
                new DriveChoreo(m_drivetrain, m_shoot, m_intake, "Comp_3_1", 0.5)
            );
        case WIGGLE:
            return new SequentialCommandGroup(
                new ResetWheels(m_drivetrain),
                new DriveChoreo(m_drivetrain, m_shoot, m_intake, "Wiggle", 1)
            );

        }
    }

    public void endAuto() {
        
    }

    public void initAuto() {
        m_drivetrain.resetOdometry();
    }
}
