package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootForTime extends Command {
   
    private final Drivetrain m_drive;
    private final Shooter m_Shoot;
    private final Intake m_intake;
    boolean shooting = false;
    double top_speed = 4200;     //RPM
    double bottom_speed = 4200;  //RPM
    double feeder_speed = 1;    //Duty Cycle
    double roller_speed = 0.12; //Duty Cycle

    double duration = 5.0;

    Timer m_timer = new Timer();

    public ShootForTime(Drivetrain drive, Shooter shooter, Intake intake, double time) {
        duration = time;

        m_drive = drive;
        m_Shoot = shooter;
        m_intake = intake;
        addRequirements(shooter, drive, intake);
    }

    @Override
    public void initialize(){
        m_timer.reset();
        m_timer.start();
    }
    
    @Override
    public void execute() {
        // top_speed = SmartDashboard.getNumber("Top Shoot Speed", top_speed) / 60;          //RPM --> Rot per sec
        // bottom_speed = SmartDashboard.getNumber("Bottom Shoot Speed", bottom_speed) / 60; //RPM --> Rot per sec
        // feeder_speed = SmartDashboard.getNumber("Shooter Feed Speed", feeder_speed);      //Duty Cycle
        // roller_speed = SmartDashboard.getNumber("Shooter Roll Speed", roller_speed);      //Duty Cycle
        
        m_Shoot.shoot(top_speed, bottom_speed, feeder_speed, roller_speed);

        if (m_timer.get() > 2) {
            
        }

        m_drive.drive(0, 0, 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        m_Shoot.shoot(0, 0, 0, 0);
    }

    @Override
    public boolean isFinished(){
        return m_timer.get() > duration;
    }
}
