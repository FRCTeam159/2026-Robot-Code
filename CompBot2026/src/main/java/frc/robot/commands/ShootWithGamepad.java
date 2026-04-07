package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootWithGamepad extends Command {
    private final Shooter m_Shoot;
    private final Intake m_Intake;
    private final XboxController m_controller;
    // private final Limelight m_limelight;

    boolean shooting = false;
    boolean shootFeeding = false;
    boolean spewing = false;
    boolean intaking = false;
    boolean out_taking = false;
    boolean arm_up = false;

    double currentDPad = -1;
    double previousDPad = -1;

    double distance;

    double top_speed = -800; // RPM
    double bottom_speed = 4110; // RPM
    double feeder_speed = 1; // Duty Cycle 1
    double roller_speed = 0.35; // Duty Cycle .35
    double intake_speed = 3000; // RPM 4000
    double winch_speed = 0.5;

    Timer m_timer;

    public ShootWithGamepad(Shooter shooter, Intake intake, XboxController controller) {
        m_timer = new Timer();
        m_Shoot = shooter;
        m_Intake = intake;
        m_controller = controller;
        addRequirements(shooter, intake);

        shooting = false;
        intaking = false;
        out_taking = false;
        arm_up = false;

        SmartDashboard.putNumber("Top Shoot Speed", top_speed);
        SmartDashboard.putNumber("Bottom Shoot Speed", bottom_speed);
        SmartDashboard.putNumber("Shooter Feed Speed", feeder_speed);
        SmartDashboard.putNumber("Shooter Roll Speed", roller_speed);
        SmartDashboard.putNumber("Intake Speed", intake_speed);

        SmartDashboard.putBoolean("Intaking", intaking);
        SmartDashboard.putBoolean("Shooting", shooting);
        SmartDashboard.putBoolean("Spewing", spewing);
    }

    public void execute() {
        SmartDashboard.putBoolean("Shooting", shooting);
        SmartDashboard.putBoolean("Spewing", spewing);
        SmartDashboard.putBoolean("Intaking", intaking);

        top_speed = SmartDashboard.getNumber("Top Shoot Speed", top_speed) / 60; // RPM --> Rot per sec
        bottom_speed = SmartDashboard.getNumber("Bottom Shoot Speed", bottom_speed) / 60; // RPM --> Rot per sec
        feeder_speed = SmartDashboard.getNumber("Shooter Feed Speed", feeder_speed); // Duty Cycle
        roller_speed = SmartDashboard.getNumber("Shooter Roll Speed", roller_speed); // Duty Cycle
        intake_speed = SmartDashboard.getNumber("Intake Speed", intake_speed) / 60; // RPM --> Rot per sec
        // distance = SmartDashboard.getNumber("distance", distance);

        // if (distance < 20) {
        // //Replace equation with whatever lines up most with real samples
        // top_speed = (1048.157 * Math.sqrt(distance)) + 3000;
        // }

        shooting = m_controller.getRightTriggerAxis() > 0.5;
        shootFeeding = m_controller.getRightBumperButton();
        out_taking = m_controller.getXButton();
        intaking = m_controller.getLeftBumperButton();

        // turns up press of d-pad into toggle
        currentDPad = m_controller.getPOV();
        if (currentDPad == 0 && previousDPad != 0) {
            arm_up = !arm_up;
        }
        previousDPad = currentDPad;

        if (shootFeeding) {
            m_Shoot.shoot(top_speed, bottom_speed, feeder_speed, roller_speed);
        } else if (shooting) {
            m_Shoot.shoot(top_speed, bottom_speed, 0, 0);
        } else if (intaking) {
            m_Shoot.shoot(0, 0, 0, roller_speed);
        } else {
            m_Shoot.shoot(0, 0, 0, 0);
        }

        if (out_taking) {
            m_Intake.intake(-intake_speed);
        } else if (intaking) {
            m_Intake.intake(intake_speed);
        } else {
            m_Intake.intake(0);
        }

        if (arm_up) {
            m_Intake.close_intake();
        } else {
            if (m_controller.getRightBumperButtonPressed()) {
                m_timer.reset();
                m_timer.start();
            }

            if (shootFeeding && m_timer.get() > 2) {
                m_Intake.change_target(m_Intake.getTarget() > 1.25 ? -0.05 : 0);
            } else {
                m_Intake.open_intake();
            }
        }

        m_Intake.spin();

        if (m_controller.getYButton()) {
            m_Intake.runWinch(0.5);
        } else if (m_controller.getAButton()) {
            m_Intake.runWinch(-0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ShootWithGamepad cancelled");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
