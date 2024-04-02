package frc.robot.commands.drivetrain;

// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

//THANK YOU LASA ROBOTICS ;;;; 
//https://github.com/lasarobotics/PurpleLib/blob/master/src/main/java/org/lasarobotics/drive/RotatePIDController.java

import java.util.HashMap;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
// import org.lasarobotics.utils.GlobalConstants;
// import org.lasarobotics.utils.PIDConstants;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;

/** Rotate PID controller */
public class RotatePIDController extends PIDController {
  private final double MIN_DEADBAND = 0.001;
  private final double MAX_DEADBAND = 0.2;
  private final double FILTER_FACTOR = 1.0 / 3.0;

  private HashMap<Double, Double> m_rotateInputMap = new HashMap<Double, Double>();
  private double m_rotateScalar;
  private double m_lookAhead;
  private double m_deadband;
  private double m_rotateRequest;
  private boolean m_isTurning;

//public static final PIDConstants DRIVE_ROTATE_PID = new PIDConstants(10.0, 0.0, 0.5, 0.0, 0.0); their code, where i got starting values

  public static final double kP = 10;
  public static final double kI = 0;
  public static final double kD = 0.5; 
  public static final double period = 1/50; //50 hz

  /**
   * Create an instance of RotatePIDController
   * @param rotateInputCurve Rotate input curve
   * @param rotateScalar Value to turn input by (degrees)
   * @param deadband Controller deadband
   * @param lookAhead Number of loops to look ahead by
   */
  public RotatePIDController(PolynomialSplineFunction rotateInputCurve, double rotateScalar, double deadband, double lookAhead) {
    super(kP, 0.0, kD);
    this.m_rotateScalar = rotateScalar;
    this.m_deadband = MathUtil.clamp(deadband, MIN_DEADBAND, MAX_DEADBAND);
    this.m_lookAhead = lookAhead;
    this.m_isTurning = false;

    // Fill turn input hashmap
    for (int i = 0; i <= 1000; i++) {
      double key = (double)i / 1000;
      double deadbandKey = MathUtil.applyDeadband(key, m_deadband);
      // Evaluate and clamp value between [0.0, +1.0]
      double value = MathUtil.clamp(rotateInputCurve.value(deadbandKey), 0.0, +1.0);
      // Add both positive and negative values to map
      m_rotateInputMap.put(+key, +value);
      m_rotateInputMap.put(-key, -value);
    }
  }

  /**
   * Returns next output of RotatePIDController
   * @param currentAngle current yaw angle of robot (degrees)
   * @param rotateRate current yaw rotate rate of robot (degrees/sec)
   * @param rotateRequest rotate request [-1.0, +1.0]
   *
   * @return optimal turn output
   */
  public double calculate(Measure<Angle> currentAngle, Measure<Velocity<Angle>> rotateRate, double rotateRequest) {
    // Filter turnRequest
    m_rotateRequest -= (m_rotateRequest - rotateRequest) * FILTER_FACTOR;

    // Start turning if input is greater than deadband
    if (Math.abs(m_rotateRequest) >= m_deadband) {
      // Get scaled turnRequest
      m_rotateRequest = Math.copySign(Math.floor(Math.abs(m_rotateRequest) * 1000) / 1000, m_rotateRequest) + 0.0;
      double scaledTurnRequest = m_rotateInputMap.get(m_rotateRequest);
      // Add delta to setpoint scaled by factor
      super.setSetpoint(currentAngle.in(Units.Degrees) + (scaledTurnRequest * m_rotateScalar));
      m_isTurning = true;
    } else {
      // When turning is complete, set setpoint to current angle
      if (m_isTurning) {
        super.setSetpoint(currentAngle.in(Units.Degrees) + (rotateRate.in(Units.DegreesPerSecond) * m_lookAhead * period));
        m_isTurning = false;
      }
    }

    return super.calculate(currentAngle.in(Units.Degrees));
  }

  /**
   * Get if robot is rotating
   * @return true if rotating
   */
  public boolean isRotating() {
    return m_isTurning;
  }
}