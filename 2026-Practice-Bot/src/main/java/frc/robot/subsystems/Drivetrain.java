package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objects.Gyro;

public class Drivetrain extends SubsystemBase {
    static public final double kDriveGearRatio = 8.14; // MK4i drive (standard)
  static public final double kTurnGearRatio = 21.429; // MK4i turn (all)

  static public boolean debug = false;
  static public boolean debug_angles = false;

  boolean m_percisionDriving = false;

  double m_driveScale = 1;
  double m_turnScale = 1;

  public static final double kWheelRadius = 2;
  public static final double kDistPerRot = (Units.inchesToMeters(kWheelRadius) * 2 * Math.PI) / kDriveGearRatio;
  public static final double kRadiansPerRot = Math.PI * 2 / kTurnGearRatio;
  
  public static final double kRobotLength = Units.inchesToMeters(24); // Waffle side length

  public static final double kFrontWheelBase = Units.inchesToMeters(18.5); // distance bewteen front wheels
  public static final double kSideWheelBase = Units.inchesToMeters(18.5); // distance beteen side wheels
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

  Gyro m_gyro = new Gyro();
  double last_heading = 0;
  Pose2d m_pose;
  boolean m_resetting = false;
  public static boolean m_fieldOriented = false;
  boolean m_disabled = true;

  private int cnt = 0;

  public boolean useOffsets = true;

    public void init() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }
   
}