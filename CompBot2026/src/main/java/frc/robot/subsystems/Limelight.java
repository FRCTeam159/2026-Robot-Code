package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {
    private DoubleTopic ta_topic = NetworkTableInstance.getDefault().getDoubleTopic("/limelight/ty");
    private DoubleSubscriber ty_sub = ta_topic.subscribe(0.0);

    private double distance = 0.0;

    public Limelight() {
        SmartDashboard.putNumber("distance", distance);
    }

    @Override
    public void periodic(){
        double ty = ty_sub.get();

        double global_angle = ty - limelight_angle;
        double global_angle_rad = Math.PI / 180 * global_angle;

        distance = (apriltag_height - limelight_height) / Math.tan(global_angle_rad);
        SmartDashboard.putNumber("distance", distance);
    }
}
