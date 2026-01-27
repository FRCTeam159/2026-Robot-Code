package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.Motor;


public class Drivetrain extends SubsystemBase {
  boolean m_enabled = false;

  double m_driveScale = 1;
  double m_turnScale = 1;

  double m_left_speed = 0.0;
  double m_right_speed = 0.0;

  private Motor left_lead_motor = new Motor(Left_lead_id, true);
  private Motor left_follow_motor = new Motor(left_follow_id, true);
  private Motor right_lead_motor = new Motor(right_lead_id, true);
  private Motor right_follow_motor = new Motor(right_follow_id, true);

  public void enable() {
    m_enabled = true;
  }

  public void disable() {
    m_enabled = false;
  }

  public void setConfig() {
    left_follow_motor.setConfig(true, false, 1, Left_lead_id);
    right_follow_motor.setConfig(false, false, 1, right_lead_id);

    left_lead_motor.setConfig(true, 1);
    right_lead_motor.setConfig(false, 1);
  }


  public void init() {
    setConfig();

    left_lead_motor.enable();
    left_follow_motor.enable();
    right_lead_motor.enable();
    right_follow_motor.enable();
  }

  public void drive(double drive_Speed, double turn_Speed) {
    if (!m_enabled) {return;}

    m_left_speed = drive_Speed * m_driveScale + turn_Speed * m_turnScale;
    m_right_speed = drive_Speed * m_driveScale - turn_Speed * m_turnScale;

    double max_speed = Math.max(Math.abs(m_left_speed), Math.abs(m_right_speed));
    if(max_speed > 1.0){
      m_left_speed /= max_speed;
      m_right_speed /= max_speed;
    }

    left_lead_motor.set(m_left_speed);
    right_lead_motor.set(m_right_speed);
  }

  
}