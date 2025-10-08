// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveWithGamepad extends Command {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_drive;
  private final XboxController m_controller;

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1.5);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1.5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(.5,-10,0);

  boolean movemode = false;
  public static double pVal = 3;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param m_Controller
   */
  public DriveWithGamepad(Drivetrain subsystem, XboxController controller) {
    m_drive = subsystem;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    SmartDashboard.putNumber("Power Value", pVal);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double now=0;//WPIUtilJNI.now() * 1e-6;
    m_xspeedLimiter.reset(now);
    m_yspeedLimiter.reset(now);
    m_rotLimiter.reset(now);

    System.out.println("DriveWithGampad started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double vx=m_controller.getLeftY();
    double vy=m_controller.getLeftX();
    double vr=m_controller.getRightX();
    boolean fo = m_drive.getFieldOriented();

    final var xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(vx, 0.2))
            * Drivetrain.kMaxVelocity;
    
    // Get the y speed or sideways/strafe speed. 
    final var ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(vy, 0.2))
            * Drivetrain.kMaxVelocity;
   
    pVal = SmartDashboard.getNumber("Power Value", 2);

    // Get the rate of angular rotation. 
   //final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(vr, 0.2))*Drivetrain.kMaxAngularVelocity;
     //final var rot = -MathUtil.applyDeadband(vr, 0.2)* Drivetrain.kMaxAngularVelocity;
     double rVal = MathUtil.applyDeadband(vr, .2);
     double sgn = rVal<0?-1:1;
     var rot = -sgn*Math.abs(Math.pow(Math.abs(rVal), pVal) * Drivetrain.kMaxAngularVelocity);

    if (m_drive.disabled()) {
      m_drive.enable();
    }
    
    if (movemode) {
      m_drive.turnAndMove(xSpeed, ySpeed);
    } else {
      m_drive.drive(xSpeed, ySpeed, rot, m_drive.isFieldOriented());
    }

    if(m_controller.getPOV() == 0){
      m_drive.setSlowDriving(true);
      
    }
    else if(m_controller.getPOV() == 180){
      m_drive.setSlowDriving(false);
    }

    if (m_controller.getRightStickButtonPressed()){
      m_drive.setFieldOriented(!m_drive.getFieldOriented()); 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DriveWithGampad cancelled");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
