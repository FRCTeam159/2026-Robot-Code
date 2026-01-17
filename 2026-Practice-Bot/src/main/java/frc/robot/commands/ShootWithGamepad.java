package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootWithGamepad extends Command {
    private final Shooter m_Shoot;
    private final XboxController m_controller;

    boolean shooting = false;

    double shoot_speed = 0.7;

    public ShootWithGamepad(Shooter shooter, XboxController controller) {

        m_Shoot = shooter;
        m_controller = controller;
        addRequirements(shooter);
    }

    public void execute() {

        if (m_controller.getRightBumperButtonPressed()) {
            if (shooting == true) {
                shooting = false;
                System.out.println("CLicked Bumper, should be false. Shooting val: " + shooting);
            }
            else {
                shooting = true;
                System.out.println("CLicked Bumper, should be true. Shooting val: " + shooting);
            }
        }

        if (shooting) {
            m_Shoot.shoot(shoot_speed);
        }
        else {
            m_Shoot.shoot(0);
        }
    }

}
