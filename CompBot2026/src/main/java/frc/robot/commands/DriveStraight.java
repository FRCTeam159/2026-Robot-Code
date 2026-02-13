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

    private static final double max_speed = 0.3;

    private double sign;

    double m_target = 1;

    public DriveStraight(Drivetrain drive, double distance) {
        sign = Math.signum(distance);

        m_target = Math.abs(distance);

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
        double s = m_drive.getAbsoluteDistance();
        double d = m_PID.calculate(s, m_target);

        d = Math.max(-max_speed, Math.min(max_speed, d));

        m_drive.drive(d * sign, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return m_PID.atSetpoint();
    }
}
