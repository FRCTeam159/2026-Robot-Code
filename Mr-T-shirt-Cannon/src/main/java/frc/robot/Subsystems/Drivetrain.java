// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparrkLowLevel.MotorType;
import com.revrobotics.config.SparkMaxConfig
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final SparkMax leftLeader = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax leftFollower = new SparkMax(2, MotorType.kBrushless);
  private final SparkMax rightLeader = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax rightFollower = new SparkMax(4, MotorType.kBrushless);
  private final DifferentialDrive m_Drive = DifferentialDrive(leftLeader, rightLeader);

  public Drivetrain() {
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();
     leftConfig.follow(1);
     rightConfig.inverted(true);

     leftFollower.configure(leftConfig, ResetMode.kReasetSafeParameters, PersistMode.kPersistParameters);
     rightFollower.configure(rightConfig, ResetMode.kReasetSafeParameters, PersistMode.kPersistParameters);
       public void drive(double left, doubleright){
      robotDrive.tankDrive(left, right);
      }

    leftLeader.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }
  

  public void drive(double left, doubleright){
    robotDrive.tankDrive(left, right);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
