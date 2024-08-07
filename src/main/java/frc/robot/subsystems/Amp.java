package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Amp extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private TalonFX motor = new TalonFX(Constants.AmperConstants.MOTOR_ID);
  
  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);


  public static enum AmperMode {
    /** At lowest mechanically possible position. */
    RESTING,

    /** Amping. */
    AMPING,

    /** Stop. */
    STOP
  }

  public Amp() {
    MotorOutputConfigs output = new MotorOutputConfigs();
    output.Inverted = InvertedValue.Clockwise_Positive;
    output.NeutralMode = NeutralModeValue.Brake;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput = output;

    // motor.getConfigurator().apply(config);

    //motion magic configs
    m_motmag.Slot = 0;

    // set slot 0 gains
    var slot0Configs = config.Slot0;
    slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0Configs.kP = 4.8;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;

    // set Motion Magic settings
    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 18; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 48; // 160 rps/s acceleration (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)

    motor.getConfigurator().apply(config, 0.050);
    motor.setPosition(0);
  }

  public void setAmpMode(AmperMode mode){
    Constants.AmperConstants.ampState = mode;
  }

  public void toggleAmpMode(){
    if(Constants.AmperConstants.ampState == AmperMode.AMPING){
      Constants.AmperConstants.ampState = AmperMode.RESTING;
    }
    else if(Constants.AmperConstants.ampState == AmperMode.RESTING){
      Constants.AmperConstants.ampState = AmperMode.AMPING;
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (Constants.AmperConstants.ampState) {
      case RESTING:
          // motor.setPosition(0);
          // motor.setControl(m_motmag.withPosition(0));
          if(motor.getPosition().getValueAsDouble() > 0.3){
            motor.setControl(m_motmag.withPosition(0));
          }
          else{
            motor.set(-0.02);
          }
          
      break;

      case AMPING:
        // if(Math.abs(motor.getPosition().getValueAsDouble() - Constants.AmperConstants.AMP_POSITION) > Constants.AmperConstants.threshold){
        //   // motor.setPosition(Constants.AmperConstants.AMP_POSITION);
        //   motor.setControl(m_motmag.withPosition(Constants.AmperConstants.AMP_POSITION));
        // }
        // else{
        //   // motor.setPosition(0);
        //   motor.setControl(m_motmag.withPosition(0));
        // }
        motor.setControl(m_motmag.withPosition(Constants.AmperConstants.AMP_POSITION + 0.2));

      break;

      case STOP:
        motor.stopMotor();
        // motor.set(0.2);
      break;
    }
  }
}
