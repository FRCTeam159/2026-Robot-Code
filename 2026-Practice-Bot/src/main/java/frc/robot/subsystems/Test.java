
package frc.robot.subsystems;

import static frc.robot.Constants.mTest;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.Motor;

public class Test extends SubsystemBase {
    private Motor motor = new Motor(mTest);

    public Test() {
        init();
    }

    public void setConfig() {
         motor.setConfig(true, 1);
    }

    public void init() {
        setConfig();

    }

    public void shoot(double speed) {
        motor.setVelocity(speed);
    }
    
}
