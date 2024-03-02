// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/** Add your docs here. */
public class j {

    public static final PS4Controller driver = new PS4Controller(0);
    public static final PS4Controller operator = new PS4Controller(1);

    //buttons
    public static JoystickButton dA = new JoystickButton(driver, PS4Controller.Button.kCross.value);
    public static JoystickButton dB = new JoystickButton(driver, PS4Controller.Button.kCircle.value);
    public static JoystickButton dY = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
    public static JoystickButton dX = new JoystickButton(driver, PS4Controller.Button.kSquare.value);
    public static JoystickButton dTouchpad = new JoystickButton(driver, PS4Controller.Button.kTouchpad.value);
    public static JoystickButton dShare = new JoystickButton(driver, PS4Controller.Button.kShare.value);
    public static JoystickButton dOptions = new JoystickButton(driver, PS4Controller.Button.kOptions.value);
    public static JoystickButton dPS = new JoystickButton(driver, PS4Controller.Button.kPS.value);
    public static JoystickButton dL3 = new JoystickButton(driver, PS4Controller.Button.kL3.value);
    public static JoystickButton dR3 = new JoystickButton(driver, PS4Controller.Button.kR3.value);

    public static JoystickButton oA = new JoystickButton(operator, PS4Controller.Button.kCross.value);
    public static JoystickButton oB = new JoystickButton(operator, PS4Controller.Button.kCircle.value);
    public static JoystickButton oY = new JoystickButton(operator, PS4Controller.Button.kTriangle.value);
    public static JoystickButton oX = new JoystickButton(operator, PS4Controller.Button.kSquare.value);
    public static JoystickButton oTouchpad = new JoystickButton(operator, PS4Controller.Button.kTouchpad.value);
    public static JoystickButton oShare = new JoystickButton(operator, PS4Controller.Button.kShare.value);
    public static JoystickButton oOptions = new JoystickButton(operator, PS4Controller.Button.kOptions.value);
    public static JoystickButton oPS = new JoystickButton(operator, PS4Controller.Button.kPS.value);
    public static JoystickButton oL3 = new JoystickButton(operator, PS4Controller.Button.kL3.value);
    public static JoystickButton oR3 = new JoystickButton(operator, PS4Controller.Button.kR3.value);


    //triggers
    public static JoystickButton oRT = new JoystickButton(operator, PS4Controller.Button.kR2.value);
    public static JoystickButton oLT = new JoystickButton(operator, PS4Controller.Button.kL2.value);
    public static JoystickButton oRB = new JoystickButton(operator, PS4Controller.Button.kR1.value);
    public static JoystickButton oLB = new JoystickButton(operator, PS4Controller.Button.kL1.value);

    public static JoystickButton dRT = new JoystickButton(driver, PS4Controller.Button.kR2.value);
    public static JoystickButton dLT = new JoystickButton(driver, PS4Controller.Button.kL2.value);
    public static JoystickButton dRB = new JoystickButton(driver, PS4Controller.Button.kR1.value);
    public static JoystickButton dLB = new JoystickButton(driver, PS4Controller.Button.kL1.value);

    //dpad
    public static POVButton dUp = new POVButton(driver, 0);
    public static POVButton dRight = new POVButton(driver, 90);
    public static POVButton dDown = new POVButton(driver, 180);
    public static POVButton dLeft = new POVButton(driver, 270);

    public static POVButton oUp = new POVButton(operator, 0);
    public static POVButton oRight = new POVButton(operator, 90);
    public static POVButton oDown = new POVButton(operator, 180);
    public static POVButton oLeft = new POVButton(operator, 270);
}
