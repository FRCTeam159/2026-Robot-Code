// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.FloatSubscriber;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
// import balls
//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.objects.Motor;
import frc.robot.utils.Averager;

public class Arm extends SubsystemBase {

  double last_heading = 0;
  static double m_navx_offset = 90; // observed gyro value when arm is horizontal
  static double shelfAngle = 180;
  static double groundAngle = 190;
  static double startAngle = 90;
  static boolean use_trap_pid=true;

  static public final double kGearRatio = 80*12.0/14.0;
  public static final double kDegreesPerRot = 360 / (kGearRatio);
  // shelf pos is 132
  // floor pos is 200

  private PIDController m_PID;
  private ProfiledPIDController m_tPID;
  static AHRS m_NAVXgyro = new AHRS(NavXComType.kUSB1);
  static AnalogInput potentiometerInput = new AnalogInput(1);

  private Motor m_armPosMotor = null;
  private Motor m_topRollerMotor = null;
  private Motor m_bottomRollerMotor = null;

  static final double START_ANGLE = 90;
  static final double MAX_ANGLE = 200-START_ANGLE;
  static final double MIN_ANGLE = START_ANGLE;
  boolean m_intake = false;
  boolean m_eject = false;
  double intakeValue = 2;
  double ejectValue = -2;

  DigitalInput m_coralSensor = new DigitalInput(1);
  DigitalInput m_coralSensor2 = new DigitalInput(0);
  //DigitalOutput m_coralState = new DigitalOutput(2);
  DigitalInput m_encoderInput = new DigitalInput(4);
  DutyCycleEncoder m_dutyCycleEncoder = new DutyCycleEncoder(m_encoderInput);

  boolean newAngle = true;
  private double armSetAngle = 0;

  private Timer m_timer = new Timer();

  double m_coralAtIntake=0;
  double m_coralAtIntake2=0;
  Averager sensor1_averager = new Averager(5);
  Averager sensor2_averager = new Averager(5);
  boolean usePOT = false;

  /**
   * Creates a new Arm.
   * 
   * @param krollers
   */
  public Arm(int armId, int bottomRollers, int topRollers) {

    m_timer.start();
  
    // SmartDashboard.putNumber("NavX", 0);
    SmartDashboard.putString("Arm", "Inactive");
    if ((Constants.testMode == Constants.test.ONEROLLER) || (Constants.testMode == Constants.test.TWOROLLERS)) {
      m_topRollerMotor = new Motor(topRollers, false);
      m_topRollerMotor.setConfig(false, 1);
      m_topRollerMotor.setPosition(0);
      m_topRollerMotor.enable();
      if (Constants.testMode == Constants.test.TWOROLLERS) {
        m_bottomRollerMotor = new Motor(bottomRollers, false);
        m_bottomRollerMotor.setConfig(false, 1);
        m_bottomRollerMotor.setPosition(0);
        m_bottomRollerMotor.enable();
      }
    } else {
      if(use_trap_pid){
        m_tPID=new ProfiledPIDController(0.01, 0, 0,
          new TrapezoidProfile.Constraints(300,200));
        m_tPID.setTolerance(1);
        m_tPID.reset(0);
      }
      else{
        m_PID = new PIDController(0.007, 0, 0);
        m_PID.setTolerance(1);
        m_PID.reset();
      }
      if (Constants.testMode == Constants.test.ARMGYRO){
        m_armPosMotor = new Motor(armId, true);
        m_armPosMotor.setConfig(false, kDegreesPerRot);
      }
      else{
        m_armPosMotor = new Motor(armId, false);
        m_armPosMotor.setConfig(true, kDegreesPerRot);
      }
      m_armPosMotor.setPosition(0);
      m_armPosMotor.enable();
    }
  }

  public boolean coralAtIntake1() {
    // return !noteSensor1.get();
    double val= m_coralSensor.get()?1:0.0;
    m_coralAtIntake = sensor1_averager.getAve(val);
    return m_coralAtIntake>0.5?false:true;
  }

  public boolean coralAtIntake2() {
    // return !noteSensor1.get();
    double val2= m_coralSensor2.get()?1:0.0;
    m_coralAtIntake2 = sensor2_averager.getAve(val2);
    return m_coralAtIntake2>0.5?false:true;
  }

  public boolean coralAtIntake() {
    if (coralAtIntake1() || coralAtIntake2())
      return true;
    else  
      return false;
  }

  public void adjustAngle(double adjustment) {
    setNewTarget(armSetAngle + adjustment);
  }

  void setNewTarget(double angle) {
    angle=angle>MAX_ANGLE?MAX_ANGLE:angle;
    angle=angle<MIN_ANGLE?MIN_ANGLE:angle;
    armSetAngle = angle;
    setPID(angle);
  }

  void setPID(double a){
    if(use_trap_pid)
      m_tPID.setGoal(a);
    else
      m_PID.setSetpoint(a);
  }
  double getPID(double c){
    if(use_trap_pid)
      return m_tPID.calculate(c);
    else
      return m_PID.calculate(c);
  }
  void setAngle() {
    double current = getAngle();
    double output = getPID(current);
    m_armPosMotor.set(output);
    String s = String.format("A:%-1.1f T:%-1.1f C:%-1.1f\n", current + START_ANGLE, armSetAngle, output);
    SmartDashboard.putString("Arm", s);
    // System.out.println(s);
  }

  public void hold() {
    System.out.println("have coral");
  }

  public void eject() {
    m_eject = true;
    System.out.println("outputting coral");
  }

  public void intake() {
    m_intake = true;
    System.out.println("picking up coral");
  }

  public boolean rollersOn() {
    return m_intake || m_eject;
  }

  public void stopRollers() {
    m_intake = false;
    m_eject = false;
  }

  public void setRollers() {
    double rollerSpeed = 0;
    // if (m_sensorDetected = false){
    //   m_sensorDetected = true;
    //   m_timer.reset();
    // }
    if (m_intake){
      if (coralAtIntake()){
        // if (m_timer.get() > 2){
        stopRollers();
        //m_sensorDetected = false;
        //}
      }
      else
        rollerSpeed = intakeValue;
    }
    else if (m_eject){
      rollerSpeed = ejectValue;
      m_timer.reset();
      if (!coralAtIntake() && m_timer.get() >= 2)
        stopRollers();
    }
    else
      rollerSpeed = 0;
    m_topRollerMotor.setVoltage(rollerSpeed);
    if (Constants.testMode == Constants.test.TWOROLLERS) 
      m_bottomRollerMotor.setVoltage(-rollerSpeed);
    String s = String.format("Eject:%b Intake:%b Speed:%1.2f", m_eject, m_intake, rollerSpeed);
    SmartDashboard.putString("Arm", s);
  }

  public void decrement(double angle) {
    // System.out.println("decrament arm by " + angle);
    adjustAngle(-angle);
  }

  public void increment(double angle) {
    // System.out.println("incrament arm by " + angle);
    adjustAngle(angle);
  }

  public void goToShelf() {
    System.out.println("going to shelf");
    setNewTarget(shelfAngle - START_ANGLE);
  }

  public void goToTest() {
    System.out.println("going to test");
    setNewTarget(startAngle);
  }

  public void goToGround() {
    System.out.println("going to ground");
    setNewTarget(groundAngle - START_ANGLE);
  }

  public void goToZero() {
    System.out.println("going to zero");
    setNewTarget(0);
  }

  public double getBoreEncoderVal() {
    double value = m_dutyCycleEncoder.get();
    return value;
    // double pStart = 3.08;
    // double pGround = 3.405;
    // double m = (startAngle-groundAngle)/(pStart-pGround); //max and min of the arm 90 and 210 over the max and min of the POT to find the slope a equation
    // double b = 0-(m * pStart);
    // double voltage = potentiometerInput.getVoltage();
    // double x = voltage * m + b;
    
    // return x - START_ANGLE;
  }

  @Override
  public void periodic() {
    //
    boolean coral = coralAtIntake();
    SmartDashboard.putBoolean("CoralDetected", coral);
    //SmartDashboard.putNumber("Pot Value", getPotentiometerValue());
    SmartDashboard.putNumber("BoreEncoder", getBoreEncoderVal());
    //m_coralState.set(coral);
    if (Constants.testMode == Constants.test.ONEROLLER || Constants.testMode == Constants.test.TWOROLLERS)
      setRollers();
    else
      setAngle();
  }

//3.08 start/90
//3.405 ground 
  public double getPotentiometerValue() {
    double pStart = 3.08;
    double pGround = 3.405;
    double m = (startAngle-groundAngle)/(pStart-pGround); //max and min of the arm 90 and 210 over the max and min of the POT to find the slope a equation
    double b = 0-(m * pStart);
    double voltage = potentiometerInput.getVoltage();
    double x = voltage * m + b;
    
    return x - START_ANGLE;
  }

  public double getAngle() {
    double angle = 0;
    if (Constants.testMode == Constants.test.ARMGYRO)
      angle = -m_NAVXgyro.getRoll() + m_navx_offset; // returned values are negative
    else if (usePOT)
      angle = getPotentiometerValue();
    else
      angle = m_armPosMotor.getPosition();
    angle = unwrap(last_heading, angle);
    last_heading = angle;
    //System.out.println(getPotentiometerValue() + " " + m_armPosMotor.getPosition());
    return angle;
  }

  public static double unwrap(double previous_angle, double new_angle) {
    double d = new_angle - previous_angle;
    d = d >= 180 ? d - 360 : (d <= -180 ? d + 360 : d);
    return previous_angle + d;
  }
}
