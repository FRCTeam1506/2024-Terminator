package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private TalonFX motor1 = new TalonFX(Constants.ShooterSubsystem.TopID);
  private TalonFX motor2 = new TalonFX(Constants.ShooterSubsystem.BottomID);

  double speed = 0.3;

  public double increment = 0.64;
      
  
  public ShooterSubsystem() {
    MotorOutputConfigs output = new MotorOutputConfigs();
    output.Inverted = InvertedValue.Clockwise_Positive;
    output.NeutralMode = NeutralModeValue.Coast;

    motor1.getConfigurator().apply(output);
    motor2.getConfigurator().apply(output);

    motor1.setInverted(false);
  }

  public void shootMax(){
    motor1.set(0.9);
    motor2.set(1);
    System.out.println("Shoot Max");
  }

  public void ampPower(){
    // if(DriverStation.getAlliance().get() == Alliance.Blue){
    //   motor1.set(0.45); //og 0.4
    //   motor2.set(0.05*1.2);
    // }
    // else{
    //   motor1.set(0.45 * 1.3); //og 0.4
    //   motor2.set(0.05*1.2 * 1.3);
    // }
<<<<<<< Updated upstream
    motor1.set(0.45); //og 0.4
    motor2.set(0.05*1.2);
=======
    motor1.set(0.6); //og 0.4 //0.45 .06
    motor2.set(0.6*1.3);  //og .5
>>>>>>> Stashed changes
  }

  public void trapPower(){
    motor1.set(Constants.ShooterSubsystem.bottomSpeed); //og 0.4
    motor2.set(Constants.ShooterSubsystem.topSpeed);//.45
  }

  public void shootSpeed(double speed){
    motor1.set(speed);
    motor2.set(speed);
  }

  public void shootRPM(){
    VelocityDutyCycle request = new VelocityDutyCycle(3000);
    motor1.setControl(request);
    motor2.setControl(request);

  }

  public void shootStop(){
    motor1.set(0);
    motor2.set(0);
  }

  public void shootIdle(){
    motor1.set(0.1);
    motor2.set(0.1);
  }


  public double getIncrement(){
    return increment;
  }

  public void increaseIncrement(){
    increment+=0.05;
  }
  public void decreaseIncrement(){
    increment-=0.05;
  }


  public double getAvgShooterSpeed(){
    return (motor1.getVelocity().getValueAsDouble() + motor2.getVelocity().getValueAsDouble())/2;
  }

  public boolean shooterUpToSpeed(){
    return getAvgShooterSpeed() > 0.8;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}