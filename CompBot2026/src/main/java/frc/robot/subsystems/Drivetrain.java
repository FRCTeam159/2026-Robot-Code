package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.DriveGyro;

public class Drivetrain extends SubsystemBase {
    static public final double kDriveGearRatio = 8.14; // MK4i drive (standard)
  static public final double kTurnGearRatio = 21.429; // MK4i turn (all)

  static public boolean debug = false;
  static public boolean debug_angles = false;

  boolean m_percisionDriving = false;

  double m_driveScale = 1;
  double m_turnScale = 0.6;

  public static final double kWheelRadius = 1.89; // o1d 1.94
  public static final double kDistPerRot = (Units.inchesToMeters(kWheelRadius) * 2 * Math.PI) / kDriveGearRatio;
  public static final double kRadiansPerRot = Math.PI * 2 / kTurnGearRatio;
  
  public static final double kRobotLength = Units.inchesToMeters(24); // Waffle side length

  public static final double kFrontWheelBase = Units.inchesToMeters(21.25); // distance bewteen front wheels
  public static final double kSideWheelBase = Units.inchesToMeters(21.25); // distance beteen side wheels
  public static final double kTrackRadius = 0.5
      * Math.sqrt(kFrontWheelBase * kFrontWheelBase + kSideWheelBase * kSideWheelBase);

  public static final double kMaxVelocity = 0.5;
  public static final double kMaxAcceleration = 1;
  public static final double kMaxAngularVelocity = Math.toRadians(720); // radians/s
  public static final double kMaxAngularAcceleration = Math.toRadians(360); // radians/s/s

  public static double dely = 0.5 * kSideWheelBase; // 0.2949 meters
  public static double delx = 0.5 * kFrontWheelBase;

  private final Translation2d m_frontLeftLocation = new Translation2d(delx, dely);
  private final Translation2d m_frontRightLocation = new Translation2d(delx, -dely);
  private final Translation2d m_backLeftLocation = new Translation2d(-delx, dely);
  private final Translation2d m_backRightLocation = new Translation2d(-delx, -dely);

  private SwerveModule m_frontLeft = new SwerveModule(kFl_Drive, kFl_Turn, kFl_Encoder, 1);
  private SwerveModule m_frontRight = new SwerveModule(kFr_Drive, kFr_Turn, kFr_Encoder, 2);
  private SwerveModule m_backLeft = new SwerveModule(kBl_Drive, kBl_Turn, kBl_Encoder, 3);
  private SwerveModule m_backRight = new SwerveModule(kBr_Drive, kBr_Turn, kBr_Encoder, 4);

  public static String chnlnames[] = { "FL", "FR", "BL", "BR" };

  private final SwerveModule[] modules = { m_frontLeft, m_frontRight, m_backLeft, m_backRight };

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private SwerveModulePosition[] m_positions = {
      new SwerveModulePosition(), new SwerveModulePosition(),
      new SwerveModulePosition(), new SwerveModulePosition() };

  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(), m_positions, new Pose2d());

  DriveGyro m_gyro = new DriveGyro(DriveGyro.gyros.PIGEON);
  double last_heading = 0;
  Pose2d m_pose;
  boolean m_resetting = false;
  public static boolean m_fieldOriented = false;
  boolean m_disabled = true;

  private int count = 0;

  public boolean useOffsets = true;


  public void setConfig() {
    m_frontRight.setConfig(false, kDistPerRot);
    m_backRight.setConfig(false, kDistPerRot);
    m_frontLeft.setConfig(true, kDistPerRot);
    m_backLeft.setConfig(true, kDistPerRot);
  }

  public void setOffsets(boolean useOffsets) {
    if (useOffsets) {
      m_frontLeft.setOffset(kFrontLeftOffset);
      m_frontRight.setOffset(kFrontRightOffset);
      m_backRight.setOffset(kBackRightOffset);
      m_backLeft.setOffset(kBackLeftOffset);
    }

  }

  public void init() {
    setOffsets(useOffsets);
    setConfig();
    enable();

    m_gyro.reset();

    SmartDashboard.putBoolean("Field Oriented", m_fieldOriented);
  }

  public boolean enabled() {
    return !m_disabled;
  }

  public boolean disabled() {
    return m_disabled;
  }
  
  public void enable() {
    m_disabled = false;
    for (int i = 0; i < modules.length; i++) {
      modules[i].enable();
    }
  }

  public void disable() {
    m_disabled = true;
    for (int i = 0; i < modules.length; i++) {
      modules[i].disable();
    }
  }

  public void setFieldOriented(boolean v){
    m_fieldOriented = v;
    if (v)
    m_gyro.reset();
    SmartDashboard.putBoolean("Field Oriented", m_fieldOriented);
  }

  public boolean isFieldOriented() {
    return m_fieldOriented;
  }

  public Rotation2d getRotation2d() {
    double angle = m_gyro.getAngle();

    // the angle returned by the gyro does not reset to 0 when it wraps around
    // angle = unwrap(last_heading, angle);

    last_heading = angle;
    return Rotation2d.fromDegrees(angle);
  }

  private void updatePositions() {
    for (int i = 0; i < modules.length; i++) {
      m_positions[i] = modules[i].getPosition();
    }
  }

  public void updateOdometry() {
    updatePositions();
    m_pose = m_odometry.update(getRotation2d(), m_positions);
  }

  public void resetOdometry(Pose2d pose) {
    m_gyro.reset();
    //resetPositions();
    m_odometry.resetPosition(getRotation2d(), m_positions, pose);
    last_heading = 0;
    m_pose = pose;
    //updateOdometry();
    System.out.println("gryo angle" + m_gyro.getAngle());
  }

  public void resetOdometry() {
      resetOdometry(new Pose2d(0, 0, new Rotation2d()));
  }


  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // target_angle += rot * m_turnScale;

    // double current_angle = getRotation2d().getDegrees();
    
    // if (Math.abs(target_angle - current_angle) < 0.5){
    //   current_angle = target_angle;
    // }//else if (Math.abs(angle_diff) < 1){
    //   angle_diff = Math.signum(angle_diff) * Math.sqrt(Math.abs(angle_diff));
    // }

    double turn_speed = rot;//turn_pid_controller.calculate(current_angle, target_angle);

    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed * m_driveScale, ySpeed * m_driveScale, turn_speed, getRotation2d())
            : new ChassisSpeeds(xSpeed * m_driveScale, ySpeed * m_driveScale, turn_speed));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxVelocity);
    for (int i = 0; i < modules.length; i++)
      modules[i].setDesiredState(swerveModuleStates[i]);

    // modules[0].setDesiredState(swerveModuleStates[0]);

    //updateOdometry();
  }

 

  public void reset() {
    m_disabled = true;
    if (debug)
      System.out.println("Drivetrain.reset");
    for (int i = 0; i < modules.length; i++) {
      modules[i].reset();
    }
    m_gyro.reset();
    last_heading = 0;
  }

  public void resetPose() {
    last_heading = 0;
    resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  public Pose2d getPose() {
    return m_pose;
  }

  public double getVelocity() {
    double speed = 0;
    for (int i = 0; i < modules.length; i++)
      speed += modules[i].getVelocity();
    return speed / modules.length;
  }

  public double getDistance() {
    double distance = 0;
    for (int i = 0; i < modules.length; i++)
      distance += modules[i].getDistance();
    return distance / modules.length;
  }

  void displayAngles() {
    if ((count % 100) == 0) {
      String str = String.format("angles fl:%-1.4f fr:%-1.4f bl:%-1.4f br:%-1.4f\n",
          m_frontLeft.getRotations(), m_frontRight.getRotations(), 
          m_backLeft.getRotations(), m_backRight.getRotations());
      SmartDashboard.putString("Wheels ", str);
    }
    count++;
  }

  public void log() {
    Pose2d pose = getPose();
    String s = String.format(" X:%-5.2f Y:%-5.2f H:%-4.1f D:%-3.1f V:%-3.1f",
        pose.getX(), pose.getY(), pose.getRotation().getDegrees(), getDistance(), getVelocity());
    SmartDashboard.putString("Pose", s);
    
    m_fieldOriented = SmartDashboard.getBoolean("Field Oriented", m_fieldOriented);
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());
    
    displayAngles();
  }

  // removes heading discontinuity at 180 degrees
  public static double unwrap(double previous_angle, double new_angle) {
    double d = new_angle - previous_angle;
    d = d >= 180 ? d - 360 : (d <= -180 ? d + 360 : d);
    return previous_angle + d;
  }

  @Override
  public void periodic() {
    updateOdometry();
    log();
  }

}