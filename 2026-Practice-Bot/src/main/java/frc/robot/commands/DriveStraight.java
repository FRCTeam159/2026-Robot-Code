package frc.robot.commands;

import frc.robot.subsystems.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveStraight extends Command {
    Drivetrain m_drive;
    PIDController m_PID;

    double m_target = 3;

    public DriveStraight(Drivetrain drive, double distance) {
        m_target = distance;

        m_PID = new PIDController(0.15, 0, 0);

        m_drive = drive;
        addRequirements(drive);
    }

    public DriveStraight(Drivetrain drive){
        this(drive, 1.0);
    }

    @Override
    public void initialize() {
        m_PID.setSetpoint(m_target);
        m_PID.setTolerance(0.1);
    }

    @Override
    public void execute() {
        double s = m_drive.getDistance();
        double d = m_PID.calculate(s, m_target);

        m_drive.drive(d, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return m_PID.atSetpoint();
    }
}
