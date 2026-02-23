package frc.robot.objects;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Motor {

    boolean upperLimitNormalyClosed = false;
    boolean lowerLimitNormalyClosed = false;

    SparkClosedLoopController velocity_controller;

    SparkMax rev_motor = null;
    RelativeEncoder rev_encoder;
    int m_chnl;
    boolean m_inverted = false;
    boolean m_enabled = true;
    SparkLimitSwitch m_upperLimit;
    SparkLimitSwitch m_lowerLimit;
    double m_dpr = 1;

    // Create a Motor wrapper for a REV SparkMax.
    // @param id CAN ID of the motor controller
    // @param isBrushed true when using a brushed motor (uses different low-level type)
    public Motor(int id, boolean isBrushed) {
        rev_motor = new SparkMax(id, isBrushed ? MotorType.kBrushed : MotorType.kBrushless);
        rev_encoder = rev_motor.getEncoder();
        m_chnl = id;
    }

    // Convenience constructor: create a brushless Motor on the given CAN ID.
    // @param id CAN ID of the motor controller
    public Motor(int id) {
        rev_motor = new SparkMax(id, MotorType.kBrushless);
        rev_encoder = rev_motor.getEncoder();
        m_chnl = id;
    }

    // Configure the forward (upper) limit switch behavior and obtain the switch object.
    // @param b true if the limit switch is normally-closed, false if normally-open
    public void setUpperLimit(boolean b) {
        upperLimitNormalyClosed = b;
        m_upperLimit = rev_motor.getForwardLimitSwitch();
    }

    // Configure the reverse (lower) limit switch behavior and obtain the switch object.
    // @param b true if the limit switch is normally-closed, false if normally-open
    public void setLowerLimit(boolean b) {
        lowerLimitNormalyClosed = b;
        m_lowerLimit = rev_motor.getReverseLimitSwitch();
    }

    // Convenience: set the upper limit switch assuming normally-open.
    public void setUpperLimit() {
        setUpperLimit(false);
    }

    // Convenience: set the lower limit switch assuming normally-open.
    public void setLowerLimit() {
        setLowerLimit(false);
    }

    // Query whether the forward (upper) limit switch is currently pressed.
    // @return true when the upper limit switch reports pressed, false otherwise
    public boolean atUpperLimit() {
        return m_upperLimit == null ? false : m_upperLimit.isPressed();
    }

    // Query whether the reverse (lower) limit switch is currently pressed.
    // @return true when the lower limit switch reports pressed, false otherwise
    public boolean atLowerLimit() {
        return m_lowerLimit == null ? false : m_lowerLimit.isPressed();
    }

    // Configure the motor controller parameters and encoder conversion factors.
    // This sets inversion, idle mode (brake/coast), and encoder conversion factors
    // based on the provided distance-per-revolution value.
    // @param isInverted true to invert motor output
    // @param isBreak true to set brake mode, false for coast
    // @param d position/velocity conversion factor (distance per motor revolution)
     public void setConfig(boolean isInverted, boolean isBreak, double d) {
        m_dpr = d;
        SparkMaxConfig config = new SparkMaxConfig();
        //config.closedLoop.pid(0, 0, 0).feedForward.kV(0.00201475);
        config
                .inverted(isInverted)
                .idleMode(isBreak ? IdleMode.kBrake : IdleMode.kCoast);
        config.encoder
                .positionConversionFactor(d)
                .velocityConversionFactor(d / 60);
        if (m_upperLimit != null){
            LimitSwitchConfig.Type upperType = upperLimitNormalyClosed?LimitSwitchConfig.Type.kNormallyClosed:LimitSwitchConfig.Type.kNormallyOpen;
            config.limitSwitch.forwardLimitSwitchType(upperType);
        }
        if (m_lowerLimit != null){
            LimitSwitchConfig.Type lowerType = lowerLimitNormalyClosed?LimitSwitchConfig.Type.kNormallyClosed:LimitSwitchConfig.Type.kNormallyOpen;
            config.limitSwitch.reverseLimitSwitchType(lowerType);
        }
        // config.closedLoop
        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // .pid(1.0, 0.0, 0.0);

        rev_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        velocity_controller = rev_motor.getClosedLoopController();
    }

    // Convenience overload: set config with default (coast) idle mode.
    // @param isInverted true to invert motor output
    // @param d position/velocity conversion factor
    public void setConfig(boolean isInverted, double d) {
        setConfig(isInverted, false, d);
    }

    // Convenience overload: not inverted, coast mode, with conversion factor d.
    // @param d position/velocity conversion factor
    public void setConfig(double d) {
        setConfig(false, false, d);
    }
    
    // Enable the software flag for this motor (does not change controller outputs).
    public void enable() {
        m_enabled = true;
    }

    // Disable this motor: clear the software flag and instruct the SparkMax to disable.
    public void disable() {
        m_enabled = false;
        rev_motor.disable();
    }

    // Return the encoder position using the configured conversion factor.
    // @return current position (units depend on conversion factor set via setConfig)
    public double getPosition() {
        return rev_encoder.getPosition();
    }

    // Set the encoder position to a specific value.
    // @param d new encoder position
    public void setPosition(double d) {
        rev_encoder.setPosition(d);
    }

    // Convenience: return the number of motor revolutions, derived from position and dpr.
    // @return rotations (position / distance-per-revolution)
    public double getRotations() {
        return getPosition() / m_dpr;
    }

    // Reset the encoder position to zero.
    public void reset() {
        rev_encoder.setPosition(0.0);
    }

    // Get the current velocity as rotations per second.
    // The REV API returns RPM by default; we divide by 60 to convert to RPS.
    // @return velocity in rotations per second
    public double getVelocity() {
        return rev_encoder.getVelocity() / 60; // rpm to rps
    }

    // Command a closed-loop target velocity (uses SparkMax closed-loop controller).
    // @param velocity target velocity (units must match controller's conversion factors)
    public void setVelocity(double velocity) {
        velocity_controller.setSetpoint(velocity, ControlType.kVelocity);
    }

    // Open-loop set the motor output (-1.0 to 1.0 typical).
    // @param speed output command
    public void set(double speed) {
        rev_motor.set(speed);
    }

    // Command the motor with a voltage setpoint.
    // @param v voltage in volts
    public void setVoltage(double v) {
        rev_motor.setVoltage(v);
    }
}

