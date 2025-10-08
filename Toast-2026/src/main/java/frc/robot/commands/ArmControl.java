// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmControl extends Command {
  public static final double ARM_MOVE_RATE = 0.5;
  Arm m_Arm;
  XboxController m_controller;

  /**
   * Creates a new ArmControl.
   * 
   * @param karm
   * @param m_controller
   */
  public ArmControl(Arm arm, XboxController controller) {
    m_Arm = arm;
    m_controller = controller;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Arm Control int");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left = m_controller.getLeftTriggerAxis();
    double right = m_controller.getRightTriggerAxis();

    if (m_controller.getAButtonPressed())
      m_Arm.goToGround();
    if (m_controller.getYButtonPressed())
      m_Arm.goToShelf();
    if (m_controller.getBButtonPressed())
      m_Arm.goToZero();
    if (m_controller.getXButtonPressed())
      m_Arm.goToTest();
    if (m_controller.getLeftBumperButtonPressed()) {
      if (m_Arm.rollersOn())
        m_Arm.stopRollers();
      else
        m_Arm.intake();
    }
    if (m_controller.getRightBumperButtonPressed()) {
      if (m_Arm.rollersOn())
        m_Arm.stopRollers();
      else
        m_Arm.eject();
    }

    else if (left > 0)
      m_Arm.decrement(left * ARM_MOVE_RATE);
    else if (right > 0)
      m_Arm.increment(right * ARM_MOVE_RATE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
