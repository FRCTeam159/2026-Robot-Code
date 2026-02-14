// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.PathData;
import frc.robot.utils.PlotUtils;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import static frc.robot.Constants.*;

// =================================================
// DrivePath: class constructor (called from RobotContainer)
// =================================================
public class DrivePath extends Command {

  double Xscale = 0.6; // Old: 1
  double Rscale = 1.4;

  ArrayList<PathData> pathdata = new ArrayList<PathData>();

  final PPHolonomicDriveController m_ppcontroller = new PPHolonomicDriveController(
      new PIDConstants(6.0, 0.0, 0), new PIDConstants(6, 0.0, 0.0));

  final HolonomicDriveController m_hcontroller = new HolonomicDriveController(new PIDController(3, 0, 0),
      new PIDController(0.5, 0, 0),
      new ProfiledPIDController(0.5, 0, 0,
          new TrapezoidProfile.Constraints(kMaxAngularVelocity * Rscale, kMaxAngularAcceleration)));

  Timer m_timer = new Timer();
  Drivetrain m_drive;
  static public boolean plot_trajectory_motion = false;
  static public boolean plot_trajectory_dynamics = false;

  static boolean debug = false;

  PathPlannerTrajectory m_pptrajectory;
  Trajectory m_trajectory;
  boolean using_pathplanner = false;
  double runtime;
  double elapsed = 0;
  int states;
  int intervals;
  public static double yPath = 0;
  public static double xPath = 1;
  public static double rPath = 0;
  static boolean m_endAtTag = false;

  double last_heading = 0;
  

  IntegerSubscriber nSub;

  int plot_type = frc.robot.utils.PlotUtils.PLOT_POSITION;

  public DrivePath(Drivetrain drive, double t, boolean use_pathplanner) {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");

    using_pathplanner = use_pathplanner;

    xPath = t;
    m_drive = drive;
    nSub = table.getIntegerTopic("NumTags").subscribe(0);
    addRequirements(drive);
  }

  // =================================================
  // initialize: Called when the command is initially scheduled.
  // =================================================
  @Override
  public void initialize() {
    xPath = SmartDashboard.getNumber("xPath", xPath);
    yPath = SmartDashboard.getNumber("yPath", yPath);
    rPath = SmartDashboard.getNumber("rPath", rPath);

    System.out.println("Starting drive path");
    //m_drive.resetPose(new Pose2d());
    plot_type = frc.robot.utils.PlotUtils.PLOT_LOCATION;

    PlotUtils.initPlot();

    if (!getTrajectory()) {
      System.out.println("failed to create Trajectory");
      return;
    }
    m_timer.start();
    m_timer.reset();

    pathdata.clear();

    elapsed = 0;

    System.out.println("runtime:" + runtime + " states:" + states + " intervals:" + intervals);
  }

  static public void setEndAtTag(boolean b){
    m_endAtTag=b;
  }

  // =================================================
  // execute: Called every time the scheduler runs while the command is scheduled
  // =================================================
  @Override
  public void execute() {

    elapsed = m_timer.get();
    Trajectory.State reference = null;
    ChassisSpeeds speeds;

    if (using_pathplanner) {
      PathPlannerTrajectoryState pstate = m_pptrajectory.sample(elapsed);
      speeds = m_ppcontroller.calculateRobotRelativeSpeeds(m_drive.getPose(), pstate);

     } else {

    reference = m_trajectory.sample(elapsed);
    reference.velocityMetersPerSecond = 0;


    double angle = Drivetrain.unwrap(last_heading, reference.poseMeters.getRotation().getDegrees());
    last_heading = angle;

    Rotation2d rot = Rotation2d.fromDegrees(angle);
    reference.poseMeters = new Pose2d(reference.poseMeters.getTranslation(), rot);
    speeds = m_hcontroller.calculate(m_drive.getPose(), reference, rot);
    
    }

    if (debug) {
      Pose2d p = m_drive.getPose();
      System.out.format(
          "%-1.3f X a:%-1.2f t:%-1.2f c:%-1.2f Y a:%-1.1f t:%-1.1f c:%-1.1f R a:%-3.1f t:%-3.1f c:%-2.1f \n", elapsed,
          p.getTranslation().getX(), reference.poseMeters.getX(), speeds.vxMetersPerSecond,
          p.getTranslation().getY(), reference.poseMeters.getY(), speeds.vyMetersPerSecond,
          p.getRotation().getDegrees(), reference.poseMeters.getRotation().getDegrees(),
          Math.toDegrees(speeds.omegaRadiansPerSecond));
    }

    m_drive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);

    if (plot_type == PlotUtils.PLOT_LOCATION)
      plotLocation(reference);
    else if (plot_type == PlotUtils.PLOT_DYNAMICS)
      plotDynamics(reference);
    else if (plot_type == PlotUtils.PLOT_POSITION)
      plotPosition(reference);
  }

  // =================================================
  // end: Called once the command ends or is interrupted.
  // =================================================
  @Override
  public void end(boolean interrupted) {
    // Autonomous.log("Drivepath.end");
    m_hcontroller.setEnabled(false);

    if (plot_type != frc.robot.utils.PlotUtils.PLOT_NONE)
      frc.robot.utils.PlotUtils.publish(pathdata, 6, plot_type);
  }

  // =================================================
  // isFinished: Returns true when the command should end
  // =================================================
  @Override
  public boolean isFinished() {
    if (m_endAtTag) {
      long n = nSub.get();
      if (n > 0) {
        System.out.println("April tag detected");
        return true;
      }
    }

    if (elapsed >= runtime) {
      System.out.println("DriveStraight Target Reached");
      return true;
    }

    return false;
  }

  // *********************** trajectory functions *******************/

  // =================================================
  // programPath: build a two-point trajectory from variables
  // =================================================

  PathPlannerTrajectory pathplannerProgramPath(){
    String path_name = "1m_Path";
    PathPlannerPath path;

    try {
    path = PathPlannerPath.fromPathFile(path_name);
    } catch (Exception e) {
      System.out.println("Error building pathplanner path: " + e);
      e.printStackTrace();
      return null;
    }

    PathPlannerTrajectory traj = new PathPlannerTrajectory(path, new ChassisSpeeds(), Rotation2d.fromDegrees(0), m_robot_config);
    return traj;
  }

  Trajectory programPath() {
    List<Pose2d> points = new ArrayList<Pose2d>();

    points.add(new Pose2d()); // start at 0,0
    points.add(new Pose2d(xPath, yPath, Rotation2d.fromDegrees(rPath)));

    TrajectoryConfig config = new TrajectoryConfig(Xscale * kMaxVelocity,
        Xscale * kMaxAcceleration);
    config.setReversed(false);

    return TrajectoryGenerator.generateTrajectory(points, config);
  }

  // =================================================
  // getTrajectory: return a selected trajectory
  // =================================================
  boolean getTrajectory() {
    if (using_pathplanner) {
      m_pptrajectory = pathplannerProgramPath();
      if (m_pptrajectory == null)
        return false;

      runtime = m_pptrajectory.getTotalTimeSeconds();
      states = m_pptrajectory.getStates().size();

      PlotUtils.setInitialPose(m_pptrajectory.sample(0).pose, kFrontWheelBase);
      m_ppcontroller.setEnabled(true);
    } else {
      m_trajectory = programPath();
      if (m_trajectory == null)
        return false;

      runtime = m_trajectory.getTotalTimeSeconds();
      states = m_trajectory.getStates().size();

      PlotUtils.setInitialPose(m_trajectory.sample(0).poseMeters, kFrontWheelBase);
      m_hcontroller.setEnabled(true);
    }

    intervals = (int) (runtime / 0.02);
    return true;

  }
  
  // *********************** plotting methods *******************/

  // =================================================
  // plotPosition: collect PathData for motion error plot
  // =================================================
  // arg[0] time of sample
  // arg[1] target pose
  // arg[2] observed pose
  // arg[3] trackwidth
  // =================================================
  void plotPosition(Trajectory.State state) {
    // PathPlanner uses holonomicRotation for heading
    Rotation2d r = state.poseMeters.getRotation();

    Pose2d p = new Pose2d(state.poseMeters.getTranslation(), r);
    state.poseMeters = p;

    PathData pd = PlotUtils.plotPosition(
        state.timeSeconds,
        state.poseMeters,
        m_drive.getPose(),
        kFrontWheelBase);
    
    pathdata.add(pd);
  }

  // =================================================
  // plotDynamics: collect PathData for dynamics error plot
  // =================================================
  // arg[0] time of sample
  // arg[1] target pose
  // arg[2] observed pose
  // arg[3] target velocity
  // arg[4] observed velocity
  // arg[5] target acceleration
  // =================================================
  void plotDynamics(Trajectory.State state) {
    PathData pd = PlotUtils.plotDynamics(
        state.timeSeconds,
        state.poseMeters, m_drive.getPose(),
        state.velocityMetersPerSecond, m_drive.getVelocity(),
        state.accelerationMetersPerSecondSq);
    
    pathdata.add(pd);
  }

  // =================================================
  // plotLocation: collect PathData for location error plot
  // =================================================
  // arg[0] time of sample
  // arg[1] target X
  // arg[2] observed X
  // arg[3] target Y
  // arg[4] observed Y
  // arg[5] target Heading
  // arg[5] observed Heading
  void plotLocation(Trajectory.State state) {
    PathData pd = new PathData();
    pd.tm = state.timeSeconds;

    Pose2d target_pose = state.poseMeters;
    Pose2d current_pose = m_drive.getPose();

    pd.d[0] = target_pose.getX();
    pd.d[1] = current_pose.getX();
    pd.d[2] = target_pose.getY();
    pd.d[3] = current_pose.getY();

    pd.d[4] = 2 * state.poseMeters.getRotation().getRadians();
    pd.d[5] = 2 * current_pose.getRotation().getRadians();

    pathdata.add(pd);
  }
}
