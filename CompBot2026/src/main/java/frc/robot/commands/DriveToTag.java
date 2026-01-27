// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TagDetector;

public class DriveToTag extends Command {
  /** Creates a new DriveToTag. */
  Drivetrain m_drive;
  PIDController m_drivePID;
  PIDController m_rotationPID;
  double m_target=0.35;
  boolean m_started=false;
  DoubleSubscriber xSub;
  DoubleSubscriber ySub;
  IntegerSubscriber nSub;
  public DriveToTag(Drivetrain drive) {
    m_drivePID = new PIDController(0.1, 0, 0);
    m_rotationPID = new PIDController(0.005, 0, 0);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    m_drive = drive;
    xSub = table.getDoubleTopic("Distance").subscribe(0.0);
    ySub = table.getDoubleTopic("Offset").subscribe(0.0);
    nSub = table.getIntegerTopic("NumTags").subscribe(0);
    addRequirements(drive);
    System.out.println("Drive To Tag");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting drive to tag");
    m_started=false;
    m_drivePID.setSetpoint(m_target);
    m_drivePID.setTolerance(0.1);
    m_rotationPID.setSetpoint(0);
    m_rotationPID.setTolerance(10);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // long n = nSub.get();
    // if(n>0){
    //   double x = xSub.get();
    //   double y = ySub.get();
    //   //System.out.println("distance = " + x + " Offset = " + y);
    //   double d = m_drivePID.calculate(x, m_target);
    //   double r = m_rotationPID.calculate(y, 0);
    //  // System.out.println("distance = " + s + " correction = " + d);
    //   m_drive.drive(-d, 0, -r, false);
    //   m_started=true;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DriveToTag.end " + interrupted);
    TagDetector.setTargeting(false);
   }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    long n = nSub.get();
    if (m_started && n==0) {
      System.out.println("Lost Tags");
      return true;
    }
    boolean atTarget = m_drivePID.atSetpoint();
    if(atTarget)
      System.out.println("DriveToTag Target Reached");
    return atTarget;
  }
}
