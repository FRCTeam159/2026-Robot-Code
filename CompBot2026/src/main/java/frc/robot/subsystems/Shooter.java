
package frc.robot.subsystems;

import static frc.robot.Constants.bottom_shooter;
import static frc.robot.Constants.hopper_roller;
import static frc.robot.Constants.shooter_feeder_1;
import static frc.robot.Constants.shooter_feeder_2;
import static frc.robot.Constants.top_shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.Motor;

public class Shooter extends SubsystemBase {
    private Motor top_shoot_motor = new Motor(top_shooter);
    private Motor bottom_shoot_motor = new Motor(bottom_shooter);
    private Motor shooter_feeder_motor_1 = new Motor(shooter_feeder_1, true);
    private Motor shooter_feeder_motor_2 = new Motor(shooter_feeder_2, true);
    private Motor roller_motor = new Motor(hopper_roller);

    public Shooter() {
        init();
    }

    public void setConfig() {
        top_shoot_motor.setConfig(false, false, 1, 0.0, 0.000);
        //keep kp 0.05
        bottom_shoot_motor.setConfig(false, false, 1, 0.04, 0.000);
        shooter_feeder_motor_1.setConfig(true, 1);
        shooter_feeder_motor_2.setConfig(true, 1);
        roller_motor.setConfig(false, 1);
    }

    public void init() {
        setConfig();

    }

    public void shoot(double top_speed, double bottom_speed, double feeder_speed, double roller_speed) {
        top_shoot_motor.setVelocity(top_speed);
        bottom_shoot_motor.setVelocity(bottom_speed);
        shooter_feeder_motor_1.set(feeder_speed);
        shooter_feeder_motor_2.set(feeder_speed);
        roller_motor.set(roller_speed);
    }
    
}
