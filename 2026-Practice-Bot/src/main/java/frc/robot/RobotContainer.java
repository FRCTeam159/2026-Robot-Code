// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Autonomous;
//import frc.robot.subsystems.TagDetector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight;
import com.ctre.phoenix6.SignalLogger;
import frc.robot.commands.ResetWheels;
import frc.robot.subsystems.Test;


public class RobotContainer {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Limelight m_limelight = new Limelight();
  private final Test m_test = new Test();
  public final Autonomous m_autonomous;
  //private final TagDetector m_Detector = new TagDetector(m_drivetrain);

  //private final Shooter m_shooter = new Shooter();

  private final DriveWithGamepad m_DriveWithGamepad = new DriveWithGamepad(m_drivetrain, m_controller, m_test);
 // private final ShootWithGamepad m_ShootWithGamepad = new ShootWithGamepad(m_shooter, m_controller);

  // private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    //Disable phoenix diagnostics automatic logging
    SignalLogger.enableAutoLogging(false);

    m_drivetrain.setDefaultCommand(m_DriveWithGamepad);
   // m_shooter.setDefaultCommand(m_ShootWithGamepad);
    m_autonomous = new Autonomous(m_drivetrain/*, m_shooter*/);
  }

  public void robotInit() {
    m_drivetrain.init();
    m_drivetrain.reset();
  }
  public void teleopInit() {
    m_drivetrain.resetOdometry();
    m_drivetrain.enable();
  }

    public void autonomousInit() {
    //m_drivetrain.resetOdometry();
    m_autonomous.initAuto();
  }

    public Command getAutonomousCommand() {
    return m_autonomous.getCommand();
  }
}
