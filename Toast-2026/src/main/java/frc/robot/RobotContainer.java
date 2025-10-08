// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.kArm;
import static frc.robot.Constants.kBottomRollers;
import static frc.robot.Constants.mTest;
import static frc.robot.Constants.kTopRollers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArmControl;
import frc.robot.commands.ClimberControl;
import frc.robot.commands.DriveWithGamepad;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TagDetector;
import frc.robot.utils.PlotServer;

public class RobotContainer {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_drivetrain = new Drivetrain();
  
  private final DriveWithGamepad m_DriveWithGamepad = new DriveWithGamepad(m_drivetrain, m_controller);

  private final TagDetector m_Detector = new TagDetector(m_drivetrain);

  private Arm m_Arm = null;
  private Climber m_Climber = null;

  public final Autonomous m_autonomous;

  public RobotContainer() {
    if (Constants.testMode == Constants.test.CLIMBER) {
      m_Climber = new Climber(mTest);
      m_Climber.setDefaultCommand(new ClimberControl(m_Climber, m_controller));
      
    } else {
      m_drivetrain.setDefaultCommand(m_DriveWithGamepad);
      m_Arm = new Arm(mTest, kBottomRollers, kTopRollers);
      m_Arm.setDefaultCommand(new ArmControl(m_Arm, m_controller));
   }
   m_autonomous = new Autonomous(m_drivetrain, m_Detector, m_Arm);
 
  }

  public void robotInit() {
    m_drivetrain.init();
    m_drivetrain.reset();
    m_Detector.start();
  }

  public void teleopInit() {
    m_autonomous.endAuto();
    m_drivetrain.resetOdometry();
    m_drivetrain.enable();
  }

  public void disabledInit() {
    m_autonomous.endAuto();
    m_drivetrain.disable();
  }

  public void autonomousInit() {
    m_autonomous.initAuto();
    m_drivetrain.resetOdometry();
  }

  public Command getAutonomousCommand() {
    return m_autonomous.getCommand();
  }
}
