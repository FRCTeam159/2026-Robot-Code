package frc.robot.commands;

import static frc.robot.Constants.shooter_feeder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

public class ShootWithGamepad extends Command {
    private final Shooter m_Shoot;
    private final Intake m_Intake;
    private final XboxController m_controller;
    //private final Limelight m_limelight;

    boolean shooting = false;
    boolean intaking = false;

    double top_speed = 1500;
    double bottom_speed = 1500;
    double feeder_speed = 1;
    double roller_speed = 0.35;

    Timer m_timer;

    public ShootWithGamepad(Shooter shooter, Intake intake, XboxController controller) {
        m_timer = new Timer();

        m_Shoot = shooter;
        m_Intake = intake;
        m_controller = controller;
        addRequirements(shooter, intake);

        SmartDashboard.putNumber("Top Shoot Speed", top_speed);
        SmartDashboard.putNumber("Bottom Shoot Speed", bottom_speed);
        SmartDashboard.putNumber("Shooter feed Speed", feeder_speed);
    }

    public void execute() {

        top_speed = SmartDashboard.getNumber("Top Shoot Speed", top_speed);
        bottom_speed = SmartDashboard.getNumber("Bottom Shoot Speed", bottom_speed);
        feeder_speed = SmartDashboard.getNumber("Shooter feed Speed", feeder_speed);

        if (m_controller.getRightBumperButtonPressed()) {
            m_timer.reset();
            m_timer.start();
            shooting = !shooting;
        }  

        if (shooting) {
            if (m_timer.get() > 2) {
                m_Shoot.shoot(top_speed, bottom_speed, feeder_speed, roller_speed);
            } else {
                m_Shoot.shoot(top_speed, bottom_speed, 0, 0);
            }
            
        }
        else {
            m_Shoot.shoot(0, 0, 0, 0);
        }

        if (m_controller.getLeftBumperButtonPressed()) {
            intaking = !intaking;
        }  

        m_Intake.intake(intaking ? 5400 : 0);
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
