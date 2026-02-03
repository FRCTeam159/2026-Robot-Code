package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class Wait extends Command{
    Timer m_timer = new Timer();
    Drivetrain m_drive;
    
    private double time = 1;

    public Wait(Drivetrain drive, double wait_time){
        time = wait_time;

        m_drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize(){
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute(){
        m_drive.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished(){
        return m_timer.get() >= time;
    }
}
