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

    private final PIDController m_drivePIDController = new PIDController(1, 0.0, 0);

    public int m_drive_chnl;
    public int m_turn_chnl;

    static boolean debug=false;

    public static boolean optimize_enabled = true;

    public static String chnlnames[] = { "FL", "FR", "BL", "BR" };

    String name;

    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.01, 0.25);

    private final PIDController m_turningPIDController = new PIDController(0.5, 0.0, 0);
   // private final ProfiledPIDController m_turningPIDController =
    //new ProfiledPIDController(0.5,0,0,);
        // new TrapezoidProfile.Constraints(
        //     Drivetrain.kMaxAngularVelocity, Drivetrain.kMaxAngularAcceleration));

    // delete later
    boolean m_enabled=false;
    
    public SwerveModule(int driveMotorChannel,int turningMotorChannel,int turningEncoderChannel,int  i) {
        m_drive_chnl=driveMotorChannel;
        m_turn_chnl=turningMotorChannel;
    
        m_driveMotor=new Motor(driveMotorChannel);
        m_turnMotor=new Motor(turningMotorChannel);
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    
        name = chnlnames[i - 1];
        
        m_turnEncoder=new Encoder(turningEncoderChannel);
      }

      public void setConfig(boolean isInverted, double distancePerRotation) {
        m_driveMotor.setConfig(isInverted, distancePerRotation);
      }

      public void setOffset(double offset){
        m_turnEncoder.setOffset(offset);
      }

      public void enable(){
        m_enabled=true;
        m_driveMotor.enable();
        m_turnMotor.enable();
      }
      public void disable(){
        m_enabled=false;
        m_driveMotor.disable();
        m_turnMotor.disable();
      }

      public void reset(){
        m_drivePIDController.reset();
      // m_turningPIDController.reset(0.0);
       m_enabled=false;
       m_driveMotor.reset();
       m_turnMotor.reset();
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

      public double getVelocity() {
        return m_driveMotor.getVelocity();
      }

      public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        // SwerveModuleState state = desiredState;  // don't optimize
        SwerveModuleState state = optimize_enabled ? SwerveModuleState.optimize(desiredState, getRotation2d()) : desiredState;
        
        double velocity=getVelocity();
        
        // Calculate the drive output from the drive PID controller.
        final double driveOutput = m_drivePIDController.calculate(m_driveMotor.getVelocity(), state.speedMetersPerSecond);
        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
        double turn_angle=getRotation2d().getRadians(); // rotations in radians
        m_targetAngle = state.angle.getRadians();
    
        final double turnOutput = -m_turningPIDController.calculate(turn_angle,state.angle.getRadians());
        final double turnFeedforward = 0;//m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
        //final double turnFeedforward = m_turnFeedforward.calculate(state.angle.getRadians());
        m_valueOfPID = turnOutput;
    
        double set_drive=driveOutput+driveFeedforward;
        double set_turn=turnOutput;//+turnFeedforward;
    

        m_driveMotor.set(set_drive);
        m_turnMotor.set(set_turn);
      }

      public void alignWheel(){
        setAngle(0);
      }

      public void resetWheel(){
        alignWheel();
      }

      public void setAngle(double a){
        m_targetAngle = a;
      }
}
