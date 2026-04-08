// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.commands.ShootWithGamepad;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;


public class RobotContainer {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Limelight m_limelight = new Limelight();
  public final Autonomous m_autonomous;

  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();

  private final DriveWithGamepad m_DriveWithGamepad = new DriveWithGamepad(m_drivetrain, m_controller);
  private final ShootWithGamepad m_ShootWithGamepad = new ShootWithGamepad(m_shooter, m_intake, m_controller);

  public RobotContainer() {
    m_drivetrain.setDefaultCommand(m_DriveWithGamepad);
    m_shooter.setDefaultCommand(m_ShootWithGamepad);
    m_autonomous = new Autonomous(m_drivetrain, m_shooter, m_intake);
  }

  public void robotInit() {
    //Disable phoenix diagnostics automatic logging
    SignalLogger.enableAutoLogging(false);
    //Disable rev automatic logging
    StatusLogger.disableAutoLogging();

    m_drivetrain.init();
    m_drivetrain.reset();
    //m_Detector.start();
  }
  public void teleopInit() {
    m_drivetrain.resetOdometry();
    m_drivetrain.enable();
  }

    public void autonomousInit() {
    m_drivetrain.resetOdometry();
    m_autonomous.initAuto();
  }

    public Command getAutonomousCommand() {
    return m_autonomous.getCommand();
  }
}
