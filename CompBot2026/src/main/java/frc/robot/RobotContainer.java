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
import frc.robot.commands.ShootWithGamepad;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;


public class RobotContainer {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_drivetrain = new Drivetrain();
  //private final Shooter m_shooter = new Shooter();

  private final DriveWithGamepad m_DriveWithGamepad = new DriveWithGamepad(m_drivetrain, m_controller);
  //private final ShootWithGamepad m_ShootWithGamepad = new ShootWithGamepad(m_shooter, m_controller);

  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    m_drivetrain.setDefaultCommand(m_DriveWithGamepad);
   // m_shooter.setDefaultCommand(m_ShootWithGamepad);

    m_chooser.setDefaultOption("Auto", "Auto");

    SmartDashboard.putData("Auto", m_chooser);
  }

  public void robotInit() {
    m_drivetrain.init();
    m_drivetrain.reset();
  }
  public void teleopInit() {
    m_drivetrain.resetOdometry();
    m_drivetrain.enable();
  }
}
