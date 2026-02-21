package frc.robot.commands;

import static frc.robot.Constants.kDistPerRot;
import static frc.robot.Constants.kDriveGearRatio;
import static frc.robot.Constants.kMaxVelocity;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.ResetWheels;
import frc.robot.Constants;
import frc.robot.commands.DrivePath;
import frc.robot.subsystems.Test;

public class DriveWithGamepad extends Command {
    private final Drivetrain m_drive;
    private final XboxController m_controller;
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1.5);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1.5);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3, -10, 0);
    double pVal = 2.0;

    private final Test m_test;

    boolean m_aligning = false;

    public DriveWithGamepad(Drivetrain drivetrain, XboxController controller, Test test) {
        m_test = test;
        m_drive = drivetrain;
        m_controller = controller;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        SmartDashboard.putNumber("Power Value", pVal);
        SmartDashboard.putNumber("Test Voltage", 0);

    }

    @Override
    public void initialize() {
        double now = 0;// WPIUtilJNI.now() * 1e-6;

        m_xspeedLimiter.reset(now);
        m_yspeedLimiter.reset(now);
        m_rotLimiter.reset(now);

        System.out.println("drive with gamepad initialized");
    }

    @Override
    public void execute() {

        double vx = m_controller.getLeftY();
        double vy = m_controller.getLeftX();
        double vr = m_controller.getRightX();
        final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(vx, 0.2))
                * Constants.kMaxVelocity;

        // Get the y speed or sideways/strafe speed.
        final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(vy, 0.2))
                * Constants.kMaxVelocity;

        pVal = SmartDashboard.getNumber("Power Value", 2);

        double rVal = MathUtil.applyDeadband(vr, 0.1);
        double sgn = rVal < 0 ? -1 : 1;
        var rot = -sgn * Math.abs(Math.pow((rVal), pVal) * Constants.kMaxAngularVelocity);
        
        m_drive.drive(xSpeed*kMaxVelocity, ySpeed*kMaxVelocity, rot, m_drive.isFieldOriented());

        if (m_controller.getRightStickButtonPressed()){
            m_drive.setFieldOriented(!m_drive.isFieldOriented()); 
            }
        
        if (m_controller.getAButtonPressed()){
            m_aligning = !m_aligning;
        }

        if (m_aligning) {
            m_drive.resetPositions();
        }


        //m_test.shoot(m_controller.getRightBumperButton() ? SmartDashboard.getNumber("Test Voltage", 0) : 0);
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
