package frc.robot.commands;

import static frc.robot.Constants.shooter_feeder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight;

public class ShootWithGamepad extends Command {
    private final Shooter m_Shoot;
    private final XboxController m_controller;
    //private final Limelight m_limelight;

    boolean shooting = false;

    double top_speed = 600;
    double bottom_speed = 600;
    double feeder_speed = 0.2;
    double roller_speed = 0.12;

    public ShootWithGamepad(Shooter shooter, XboxController controller) {

        m_Shoot = shooter;
        //m_limelight = limelight;
        m_controller = controller;
        addRequirements(shooter);

        SmartDashboard.putNumber("Top Shoot Speed", top_speed);
        SmartDashboard.putNumber("Bottom Shoot Speed", bottom_speed);
        SmartDashboard.putNumber("Shooter feed Speed", feeder_speed);
    }

    public void execute() {

        top_speed = SmartDashboard.getNumber("Top Shoot Speed", top_speed);
        bottom_speed = SmartDashboard.getNumber("Bottom Shoot Speed", bottom_speed);
        feeder_speed = SmartDashboard.getNumber("Shooter feed Speed", feeder_speed);

        if (m_controller.getRightBumperButtonPressed()) {
            //System.out.println("top: " + top_speed + ", bottom: " + bottom_speed);
            shooting = !shooting;
        }  

        if (shooting) {
            m_Shoot.shoot(top_speed, bottom_speed, feeder_speed, roller_speed);
        }
        else {
            m_Shoot.shoot(0, 0, 0, 0);
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
