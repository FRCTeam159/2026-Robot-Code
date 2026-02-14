// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.*;

import java.util.ArrayList;
import java.util.List;

// import com.pathplanner.*;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class DrivePathPlanner extends Command {

    private PathPlannerPath pathFile;
    private List<PathPoint> pathPoints;
    
    Drivetrain m_drive;

    private String file;
    private Trajectory m_trajectory;

    double maxV;
    double maxA;



    public DrivePathPlanner(Drivetrain drive) {
        m_drive = drive;
        addRequirements(drive);
    }

    //@Override
    // public void initialize() {
    //     maxV = kMaxVelocity;
    //     maxA = kMaxAcceleration;

    //     m_trajectory = getTrajectory();
    // }

    // Trajectory getTrajectory() {
    //     file = "1m_Path";
    //     pathFile = PathPlannerPath.fromPathFile(file);
    //     pathPoints = pathFile.getAllPathPoints();
    //     ArrayList<Rotation2d> rotationList = new ArrayList<>();
    //     for(int i=0;i<pathPoints.size();i++){
    //         if (pathPoints.get(i).rotationTarget != null) {
    //             rotationList.add(pathPoints.get(i).rotationTarget.rotation());
    //     }
    //   }

    //     return new Trajectory(stateList);
    // }

}
