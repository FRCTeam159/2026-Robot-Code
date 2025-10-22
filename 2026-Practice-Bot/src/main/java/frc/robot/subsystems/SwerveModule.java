package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.objects.Motor;
import frc.robot.objects.Encoder;

public class SwerveModule {
    private Motor m_driveMotor = null;
    private Motor m_turnMotor = null;
    private Encoder m_turnEncoder=null;
    double m_valueOfPID=0;
    double m_targetAngle=0;

    public int m_drive_chnl;
    public int m_turn_chnl;

    public static String chnlnames[] = { "FL", "FR", "BL", "BR" };

    String name;

    private final ProfiledPIDController m_turningPIDController =
    new ProfiledPIDController(0.5,0,0,
        new TrapezoidProfile.Constraints(
            Drivetrain.kMaxAngularVelocity, Drivetrain.kMaxAngularAcceleration));

    public SwerveModule(int driveMotorChannel,int turningMotorChannel,int turningEncoderChannel,int  i) {
        m_drive_chnl=driveMotorChannel;
        m_turn_chnl=turningMotorChannel;
    
        m_driveMotor=new Motor(driveMotorChannel);
        m_turnMotor=new Motor(turningMotorChannel);
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    
        name = chnlnames[i - 1];
        
        m_turnEncoder=new Encoder(turningEncoderChannel);
      }

      public double getRotations(){
        return m_turnEncoder.getPosition();
      }

      public double heading(){
        return 2*Math.PI*getRotations();
      }

      public Rotation2d getRotation2d() {
        return Rotation2d.fromRadians(heading());
      }
      
      public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_driveMotor.getPosition(),  getRotation2d());
      }
    
      public double getDistance(){
        return m_driveMotor.getPosition();
      }
}
