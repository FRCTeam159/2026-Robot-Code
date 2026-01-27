
package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.Motor;

public class Shooter extends SubsystemBase {
    private Motor shoot_motor = new Motor(shoot_id, true);

    public Shooter() {
        init();
    }

    public void setConfig() {
        shoot_motor.setConfig(false, 1);
    }

    public void init() {
        setConfig();
    }

    public void shoot(double speed) {
        shoot_motor.set(speed);
    }
    
}
