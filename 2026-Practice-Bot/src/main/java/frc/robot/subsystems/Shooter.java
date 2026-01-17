
package frc.robot.subsystems;

import static frc.robot.Constants.bottom_shooter;
import static frc.robot.Constants.top_shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.Motor;

public class Shooter extends SubsystemBase {
    private Motor top_shoot_motor = new Motor(top_shooter);
    private Motor bottom_shoot_motor = new Motor(bottom_shooter);

    // public void setConfig() {
    //     bottom_shoot_motor.setConfig(true, 0);
    // }

    // public void init() {
    //     setConfig();

    // }

    public void shoot(double speed) {
        top_shoot_motor.set(speed);
        bottom_shoot_motor.set(-speed);
    }
    
}
