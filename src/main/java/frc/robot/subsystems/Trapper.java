// in honor of scrapper the cat

// 2002 - 2019

// rest in peace

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Trapper extends SubsystemBase {
  /** Creates a new Angler. */
  TalonFX vertical = new TalonFX(Constants.TrapperSubsystem.TRAPPER_VERTICAL_ID);
  TalonFX shooter = new TalonFX(Constants.TrapperSubsystem.TRAPPER_SHOOTER_ID);

  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

  public Trapper() {
    // robot init
    var talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    //talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    //talonFXConfigs.CurrentLimits.StatorCurrentLimit = 60;


    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0Configs.kP = 4.8;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 160; // 160 rps/s acceleration (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)

    vertical.getConfigurator().apply(talonFXConfigs, 0.050);
    shooter.getConfigurator().apply(talonFXConfigs, 0.050);
    vertical.setPosition(0);
  }

  public void setPosition(double position){
    m_motmag.Slot = 0;
    vertical.setControl(m_motmag.withPosition(position));
  }

  public void shootTrap(){
    shooter.set(-0.3); //-0.3 //0.45
  }

  public void intake(){
    shooter.set(1);//0.3
  }

  public void trapPosition(){
    m_motmag.Slot = 0;
    vertical.setControl(m_motmag.withPosition(4.6)); //5.1 too high //4.85 too high
  }

  public void sendTrapperHome(){
    m_motmag.Slot = 0;
    vertical.setControl(m_motmag.withPosition(5.1));
  }


  public void HP(){
    m_motmag.Slot = 0;
    vertical.setControl(m_motmag.withPosition(2));
  }

  public void up(){
    vertical.set(0.3);
  }

  public void down(){
    vertical.set(-0.3);
  }


  public void stopTrapper(){
    shooter.set(0);
    vertical.stopMotor();
  }

  public void stopIntake(){
    shooter.set(0);
    shooter.stopMotor();
  }

  public void verticalZero(){
    vertical.setPosition(0);
  }

  public void setAmpPosition(){
    setPosition(Constants.TrapperSubsystem.AmpPosition);
  }

  public double getPos(){
    return vertical.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
