package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class ShootWithGamepad extends Command {
    private final Shooter m_Shoot;
    private final Intake m_Intake;
    private final XboxController m_controller;

    private boolean shooting = false;
    private boolean intaking = false;

    private double shoot_Speed = 0.2;
    private final double intake_Speed = -0.7;

    public ShootWithGamepad(Shooter shooter, Intake intake, XboxController controller) {
        m_Shoot = shooter;
        m_Intake = intake;
        m_controller = controller;
        addRequirements(shooter);

        SmartDashboard.putNumber("Shoot Speed", shoot_Speed);
    }

    public void execute() {
        shoot_Speed = SmartDashboard.getNumber("Shoot Speed", shoot_Speed);

        if (m_controller.getRightBumperButtonPressed()) {
            shooting = !shooting;
        }  

        if (shooting) {
            m_Shoot.shoot(shoot_Speed);
        } else {
            m_Shoot.shoot(0);
        }

        if (m_controller.getLeftBumperButtonPressed()) {
            intaking = !intaking;
        }

        m_Intake.set(intaking ? intake_Speed : 0);
    }

}
