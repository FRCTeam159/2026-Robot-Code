package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.StringSubscriber;
import frc.robot.objects.LimelightHelpers;
import frc.robot.objects.LimelightHelpers.RawFiducial;

public class Limelight extends SubsystemBase {

    private NetworkTable tag_table = NetworkTableInstance.getDefault().getTable("datatable");
    private DoublePublisher distance_pub = tag_table.getDoubleTopic("Distance").publish();
    private DoublePublisher offset_pub = tag_table.getDoubleTopic("Offset").publish();
    private DoublePublisher num_tag_pub = tag_table.getDoubleTopic("NumTags").publish();

    private double distance = 0.0;
  

    public Limelight() {
        SmartDashboard.putNumber("distance", distance);
    }

    public static double getDistance(double ty){
        //account for camera angle and convert to radians
        double global_angle = ty + limelight_angle;
        double global_angle_rad = (Math.PI / 180) * global_angle;

        //calculate distance using the opposite side of a triangle;
        return (apriltag_height - limelight_height) / Math.tan(global_angle_rad);
    }

    @Override
    public void periodic(){

        double tx = LimelightHelpers.getTX("limelight");
        double tv = LimelightHelpers.getTV("limelight") ? 1 : 0;

        //vertical angle to center of april tag on the camera
        double ty = LimelightHelpers.getTY("limelight");

        distance = getDistance(ty);

        distance_pub.set(distance);
        offset_pub.set(tx);
        num_tag_pub.set(tv);

        RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials("");

        String s = "" + rawFiducials.length;

        for (RawFiducial target : rawFiducials){
        s += String.format(" %d: %.2f,", target.id, getDistance(target.tync));
        }

        SmartDashboard.putString("tags", s);


        SmartDashboard.putNumber("distance", distance * 39.37);
        SmartDashboard.putNumber("angle", ty);
        SmartDashboard.putNumber("Offset", tx);
    }
}
