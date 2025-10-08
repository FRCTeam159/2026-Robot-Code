// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.mTest;

import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.objects.Motor;
import pabeles.concurrency.ConcurrencyOps.Reset;
import edu.wpi.first.wpilibj.DigitalOutput;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   * 
   * @param kclimber
   */

  static double lowValue = -0.7;
  static double highValue = 0;// Inches
  private double m_setPoint = 0;
  double lowerPower = -1.5;
  double raisePower = 1.5;
  static public final double kRotToIn = 0.1;
  static public final double kGearRatio = 250;
  static public final double kInchesPerRot = kRotToIn / kGearRatio;
  boolean m_raising = false;
  boolean m_lowering = false;
  boolean m_atUpperTarget = false;
  boolean m_atLowerTarget = false;

  boolean noPID = true;

  DigitalOutput m_climbState = new DigitalOutput(3);


  private final PIDController m_PID = new PIDController(0.01, 0, 0);

  private Motor m_ClimberMotor;

  public Climber(int kclimber) {
    SmartDashboard.putString("Climber", "Inactive");
    m_ClimberMotor = new Motor(mTest);
    m_ClimberMotor.setUpperLimit(true);
    m_ClimberMotor.setLowerLimit(true);
    m_ClimberMotor.setConfig(false, true, kInchesPerRot);
    m_ClimberMotor.setPosition(0);
    m_PID.setTolerance(0.2);
    
    m_climbState.set(false);
  }

  public void raise() {
    // Raises the climber claw
    System.out.println("Raising the Climber");
    setTargetHeight(highValue);
    reset();
    m_raising = true;
    m_climbState.set(false);
  }

  public boolean raising() {
    return m_raising;
  }

  public boolean lowering() {
    return m_lowering;
  }

  public void lower() {
    // Lowers the climber claw
    System.out.println("Lowering the Climber");
    setTargetHeight(lowValue);
    reset();
    m_lowering = true;
    m_climbState.set(false);
  }

  public void stop() {
    if (!noPID)
    setTargetHeight(getHeight());
    
    System.out.println("Stop");
    reset();
  }

  void setTargetHeight(double d) {
    m_setPoint = d;
  }

  void setHeight() {
    double output = 0;
    double current = getHeight();

    if (noPID) {
      if (m_raising)
        output = raisePower;
      else if (m_lowering)
        output = lowerPower;
      else
        output = 0;

    } else {
      m_PID.setSetpoint(m_setPoint);
      m_PID.calculate(current);
    }

    m_ClimberMotor.set(output);
    String s = String.format("A:%-1.5f T:%-1.5f C:%-1.1f R:%b L:%b\n", current, m_setPoint, output, m_raising, m_lowering);
    SmartDashboard.putString("Climber", s);
    //System.out.println(s);
  }

  public double getHeight() {
    double height = m_ClimberMotor.getPosition();
    return height;
  }

  public boolean atUpperTarget() {
    //if (m_ClimberMotor.atUpperLimit())
    //return true;
    if (noPID)
      return getHeight() > highValue?true:false;
    else
      return m_PID.atSetpoint();// || m_ClimberMotor.atUpperLimit();
  }

  public boolean atLowerTarget() {
    //if (m_ClimberMotor.atLowerLimit())
    //return true;
    if (noPID)
      return getHeight() < lowValue?true:false;
    else
      return m_PID.atSetpoint();// || m_ClimberMotor.atLowerLimit();
  }

  public void reset() {
    m_atUpperTarget = false;
    m_atLowerTarget = false;
    m_raising = false;
    m_lowering = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setHeight();
    if (m_raising) {
      if (m_ClimberMotor.atUpperLimit()){
        stop();
        //setTargetHeight(lowValue);
      }else if (atUpperTarget()) {
        m_atUpperTarget = true;
        m_raising = false;
        m_climbState.set(true);
      }
    }
    if (m_lowering) {
      if (m_ClimberMotor.atLowerLimit()){
        stop();
        //setTargetHeight(highValue);
      }else if (atLowerTarget()) {
        m_atLowerTarget = true;
        m_lowering = false;
      }
    }
    SmartDashboard.putBoolean("UpperTarget", m_atUpperTarget);
    SmartDashboard.putBoolean("LowerTarget", m_atLowerTarget);
  }
}
