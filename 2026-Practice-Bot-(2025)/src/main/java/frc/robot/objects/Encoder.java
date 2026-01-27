package frc.robot.objects;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.measure.Angle;

public class Encoder {
    CANcoder can_encoder;
    CANcoderConfiguration config = new CANcoderConfiguration();

    double offset=0;

    public Encoder(int id) {
        //creates can_encoder and configures it
        can_encoder = new CANcoder(id);
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        can_encoder.getConfigurator().apply(config);
    }

    public void setOffset(double new_offset) {
        //Sets offset and reconfigures the encoder
        offset = new_offset;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetSensor.MagnetOffset = -offset;
        can_encoder.getConfigurator().apply(config);
    }
    
    public double getPosition() {
        StatusSignal<Angle> val = can_encoder.getPosition(true);
        double pos =val.getValueAsDouble();
        return pos;
    }

    public double getAbsolutePosition() {// return rotations
        StatusSignal<Angle> val = can_encoder.getAbsolutePosition(true);
        return val.getValueAsDouble()-offset; 
    }

}
