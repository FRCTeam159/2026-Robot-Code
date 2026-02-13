package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Limelight extends SubsystemBase {
    // private NetworkTable limelight_table = NetworkTableInstance.getDefault().getTable("limelight");
    // private DoubleSubscriber tx_sub = limelight_table.getDoubleTopic("tx").subscribe(0.0);
    // private DoubleSubscriber ty_sub = limelight_table.getDoubleTopic("ty").subscribe(0.0);
    // private DoubleSubscriber ta_sub = limelight_table.getDoubleTopic("ta").subscribe(0.0);

    // private NetworkTable tag_table = NetworkTableInstance.getDefault().getTable("datatable");
    // private DoublePublisher distance_pub = tag_table.getDoubleTopic("Distance").publish();
    // private DoublePublisher offset_pub = tag_table.getDoubleTopic("Offset").publish();
    // private DoublePublisher num_tag_pub = tag_table.getDoubleTopic("NumTags").publish();

    private double distance_coefficient = 5;

    private double distance = 0.0;

    public Limelight() {
        SmartDashboard.putNumber("distance", distance);
    }

    @Override
    public void periodic(){
        //horizontal offset to center of limelight
        double tx = LimelightHelpers.getTX("limelight");

        //vertical angle to center of april tag on the camera
        double ty = LimelightHelpers.getTY("limelight");

        //account for camera angle and convert to radians
        double global_angle = ty - limelight_angle;
        double global_angle_rad = (Math.PI / 180) * global_angle;

        //calculate distance using the opposite side of a triangle;
        distance = (apriltag_height - limelight_height) / Math.tan(global_angle_rad);

        //area the april tag takes on the screen
        double ta = LimelightHelpers.getTA("limelight");

        SmartDashboard.putNumber("distance", distance * 39.37);
        SmartDashboard.putNumber("ta distance", distance_coefficient / Math.sqrt(ta));
    }
}