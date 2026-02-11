
package frc.robot.subsystems;

import static frc.robot.Constants.bottom_shooter;
import static frc.robot.Constants.shooter_feeder;
import static frc.robot.Constants.top_shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.Motor;

public class Shooter extends SubsystemBase {
    private Motor top_shoot_motor = new Motor(top_shooter);
    private Motor bottom_shoot_motor = new Motor(bottom_shooter);
    private Motor shooter_feeder_motor = new Motor(shooter_feeder);

    public Shooter() {
        init();
    }

    public void setConfig() {
        top_shoot_motor.setConfig(false, 1);
        bottom_shoot_motor.setConfig(false, 1);
        shooter_feeder_motor.setConfig(true, 1);

    }

    public void init() {
        setConfig();

    }

    public void shoot(double top_speed, double bottom_speed, double feeder_speed) {
        top_shoot_motor.set(top_speed);
        bottom_shoot_motor.set(bottom_speed);
        shooter_feeder_motor.set(feeder_speed);
    }
    
}
