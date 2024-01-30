// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/** Add your docs here. */
public class j {

    public static final PS4Controller driver = new PS4Controller(0); // My joystick
    public static final PS4Controller operator = new PS4Controller(1); // My joystick

    public static JoystickButton dX = new JoystickButton(driver, PS4Controller.Button.kCross.value);
    public static JoystickButton dCircle = new JoystickButton(driver, PS4Controller.Button.kCircle.value);
    public static JoystickButton dTriangle = new JoystickButton(driver, PS4Controller.Button.kCircle.value);

    public static POVButton dUp = new POVButton(driver, 0);
    public static POVButton dRight = new POVButton(driver, 90);
    public static POVButton dDown = new POVButton(driver, 180);
    public static POVButton dLeft = new POVButton(driver, 270);

    public static POVButton oUp = new POVButton(operator, 0);
    public static POVButton oRight = new POVButton(operator, 90);
    public static POVButton oDown = new POVButton(operator, 180);
    public static POVButton oLeft = new POVButton(operator, 270);

}
