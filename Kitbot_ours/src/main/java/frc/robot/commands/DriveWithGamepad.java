package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveWithGamepad extends Command {
    private final Drivetrain m_drive;
    private final XboxController m_controller;
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1.5);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1.5);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3, -10, 0);
    double pVal = 2.0;

    public DriveWithGamepad(Drivetrain drivetrain, XboxController controller) {

        m_drive = drivetrain;
        m_controller = controller;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        SmartDashboard.putNumber("Power Value", pVal);

    }

    @Override
    public void initialize() {
        double now = 0;// WPIUtilJNI.now() * 1e-6;
        m_xspeedLimiter.reset(now);
        m_yspeedLimiter.reset(now);
        m_rotLimiter.reset(now);
    }

    @Override
    public void execute() {
        double vx = m_controller.getLeftY();
        double vr = m_controller.getRightX();

        double drive_Speed = MathUtil.applyDeadband(vx, 0.2);
        double turn_Speed = MathUtil.applyDeadband(vr, 0.2);
        
        m_drive.drive(drive_Speed, turn_Speed);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DriveWithGampad cancelled");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}