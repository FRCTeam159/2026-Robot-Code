
package frc.robot.subsystems;

import static frc.robot.Constants.bottom_shooter;
import static frc.robot.Constants.top_shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.Motor;

public class Shooter extends SubsystemBase {
    private Motor motor = new Motor(mTest);

    public Shooter() {
        init();
    }

    public void setConfig() {
         motor.setConfig(true, 1);
    }

    public void init() {
        setConfig();

    }

    public void shoot(double speed) {
        motor.set(speed);
    }
    
}
