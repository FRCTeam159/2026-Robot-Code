
package frc.robot.subsystems;

import static frc.robot.Constants.kDistPerRot;
import static frc.robot.Constants.mTest;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.Motor;

public class Test extends SubsystemBase {
    Motor motor;
    
    Drivetrain m_drive;

    public Test(Drivetrain drive) {
        m_drive = drive;

        motor = drive.m_backLeft.m_driveMotor;

        setConfig();
    }

    public void setConfig() {
        motor.setConfig(true, 1);
    }

    public void shoot(double speed) {
        motor.setVelocity(speed);
    }
    
}
