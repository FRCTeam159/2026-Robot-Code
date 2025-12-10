// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.objects;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import static frc.robot.Constants.*;
import com.ctre.phoenix6.hardware.Pigeon2;

public class DriveGyro {
  static public enum gyros {
    FRC450,
    PIGEON
  } ;
  gyros gyro_type=gyros.FRC450;
  String gyro_name;
  ADXRS450_Gyro adx450=null;
  boolean bnoWasInitialized = false;
  Pigeon2 pigeon=null;

  
  /** Creates a new Gyro. */
  public DriveGyro(gyros type) {
    gyro_type=type;
    switch(type){
      default:
      case FRC450:
        adx450= new ADXRS450_Gyro();
        gyro_name="FRC450";
        break;
      case PIGEON:
        pigeon = new Pigeon2(kPigeonCanId);
        break;
    }
  }
  public gyros getType(){
    return gyro_type;
  }
  public String toString(){
    return gyro_name;
  }
  
  public void reset() {
    switch(gyro_type){
      default:
      case FRC450:
        adx450.reset();
        break;
      case PIGEON:
        pigeon.reset();
        break;
    }
  }

  public double getAngle() {
    switch(gyro_type){
      default:
      case FRC450:
        return -adx450.getAngle();
      case PIGEON:
        return pigeon.getRotation2d().getDegrees();
    }
  }
}
