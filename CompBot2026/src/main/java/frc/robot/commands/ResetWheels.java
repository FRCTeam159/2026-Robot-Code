package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

public class ResetWheels extends Command {
    Drivetrain m_drive;
    
    Timer m_timer = new Timer();

    public ResetWheels(Drivetrain drive) {
        m_drive = drive;

        addRequirements(m_drive);
    }

    @Override
    public void initialize(){
        SwerveModule.optimize_enabled = false;

        m_drive.resetPositions();
        m_drive.resetOdometry();

        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        m_drive.drive(1e-5, 0, 0, false);
    }

    @Override
    public boolean isFinished(){
        return m_timer.get() > 1;
    }

    @Override
    public void end(boolean interrupted){
        SwerveModule.optimize_enabled = true;
    }
}