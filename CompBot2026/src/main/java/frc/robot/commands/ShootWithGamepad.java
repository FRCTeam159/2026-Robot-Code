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
    //private final Limelight m_limelight;

    boolean shooting = false;
    boolean intaking = false;

    double top_speed = 5300;    //RPM
    double bottom_speed = 5300; //RPM
    double feeder_speed = 1;    //Duty Cycle
    double roller_speed = 0.35; //Duty Cycle
    double intake_speed = 4000; //RPM

    Timer m_timer;

    public ShootWithGamepad(Shooter shooter, Intake intake, XboxController controller) {
        m_timer = new Timer();

        m_Shoot = shooter;
        m_Intake = intake;
        m_controller = controller;
        addRequirements(shooter, intake);

        SmartDashboard.putNumber("Top Shoot Speed", top_speed);
        SmartDashboard.putNumber("Bottom Shoot Speed", bottom_speed);
        SmartDashboard.putNumber("Shooter Feed Speed", feeder_speed);
        SmartDashboard.putNumber("Shooter Roll Speed", roller_speed);
        SmartDashboard.putNumber("Intake Speed", intake_speed);
    }

    public void execute() {

        top_speed = SmartDashboard.getNumber("Top Shoot Speed", top_speed) / 60;           //RPM --> Rot per sec
        bottom_speed = SmartDashboard.getNumber("Bottom Shoot Speed", bottom_speed) / 60;  //RPM --> Rot per sec
        feeder_speed = SmartDashboard.getNumber("Shooter Feed Speed", feeder_speed);       //Duty Cycle
        roller_speed = SmartDashboard.getNumber("Shooter Roll Speed", roller_speed);       //Duty Cycle
        intake_speed = SmartDashboard.getNumber("Intake Speed", intake_speed) / 60;        //RPM --> Rot per sec

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
            m_Shoot.shoot(0, 0, 0, intaking ? intake_speed : 0);
        }

        if (m_controller.getLeftBumperButtonPressed()) {
            intaking = !intaking;
        }  
        
        m_Intake.intake(intaking ? intake_speed : 0);
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
