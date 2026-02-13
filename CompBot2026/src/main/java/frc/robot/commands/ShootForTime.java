package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootForTime extends Command {
   
    //private final Shooter m_Shoot;
    boolean shooting = false;
    double top_speed = 0.2;
    double bottom_speed = 0.2;

    Timer m_timer = new Timer();

    // public ShootForTime(/*Shooter shooter*/) {
    //     //m_Shoot = shooter;
    //     //addRequirements(shooter);
    // }

    @Override
    public void initialize(){
        m_timer.reset();
        m_timer.start();
        System.out.println("Shooting");
    }
    
    @Override
    public void execute() {
        top_speed = SmartDashboard.getNumber("Top Shoot Speed", top_speed);
        bottom_speed = SmartDashboard.getNumber("Bottom Shoot Speed", bottom_speed);
        
        //m_Shoot.shoot(top_speed, bottom_speed);
    }

    @Override
    public void end(boolean interrupted) {
        //m_Shoot.shoot(0, 0);
        System.out.println("Stopped Shooting");
    }

    @Override
    public boolean isFinished(){
        return m_timer.get() > 5;
    }
}
