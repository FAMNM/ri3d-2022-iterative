// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Driving Controls
  XboxController xboxLeft;
  XboxController xboxRight;

  // DriveTrain Motors
  WPI_VictorSPX leftDrive1;
  WPI_VictorSPX leftDrive2;
  WPI_VictorSPX rightDrive1;
  WPI_VictorSPX rightDrive2;
  WPI_VictorSPX shooterWheel1;
  WPI_VictorSPX shooterWheel2;
  
  MotorControllerGroup leftDrive;
  MotorControllerGroup rightDrive;

  DifferentialDrive differentialDrive;

  Servo servoMotor;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    new Thread(() -> {
      UsbCamera camera = CameraServer.startAutomaticCapture();
      camera.setResolution(320, 240);
      // camera.setResolution(10, 10);

      CvSink cvSink = CameraServer.getVideo();
      CvSource outputStream = CameraServer.putVideo("Blur", 320, 240);
      // CvSource outputStream = CameraServer.putVideo("Blur", 10, 10);
      
      Mat source = new Mat();
      Mat hsvmat = new Mat();
      Mat coreOutput = new Mat();
      Mat rgbmat = new Mat();
      
      while(!Thread.interrupted()) {
        if(cvSink.grabFrame(source) == 0) {
          continue;
        }

        Imgproc.cvtColor(source, hsvmat, Imgproc.COLOR_RGB2HSV_FULL);
        
        // Imgproc.GaussianBlur(hsvmat, blurOutput, new Size(9, 9), 0, 0);
        // Imgproc.blur(hsvmat, blurOutput, new Size(9,9));
        // Imgproc.erode(blurOutput, erodeOutput, new Size(5,5));
        // Imgproc.medianBlur(hsvmat, blurOutput, 9);

        Core.inRange(hsvmat, new Scalar(120,Integer.MIN_VALUE,Integer.MIN_VALUE), new Scalar(140,Integer.MAX_VALUE,Integer.MAX_VALUE), coreOutput);
        // 115 to 130
        // 105 to 130 was also good

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

        Imgproc.findContours(coreOutput, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        double maxArea = 0;
        MatOfPoint maxContour = new MatOfPoint();
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {

          MatOfPoint wrapper = each.next();
          double area = Imgproc.contourArea(wrapper);
          if (area > maxArea) {
            maxArea = area;
            maxContour = wrapper;
          }
        }

        SmartDashboard.putNumber("DB/String 0", 160);
        SmartDashboard.putNumber("DB/String 1", 160);

        if (maxArea != 0) {
          Moments m = Imgproc.moments(maxContour, false);
          int x = (int) (m.get_m10() / m.get_m00());
          int y = (int) (m.get_m01() / m.get_m00());
          Imgproc.circle(source, new Point(x, y), 10, new Scalar(255, 0, 0));
          SmartDashboard.putNumber("DB/String 0", x);
          SmartDashboard.putNumber("DB/String 1", y);
        }
        
        
        


        outputStream.putFrame(source);
      }
    }).start();

    xboxLeft = new XboxController(0);
    // xboxRight = new XboxController(1);

    leftDrive1 = new WPI_VictorSPX(6);
    leftDrive2 = new WPI_VictorSPX(7);
    rightDrive1 = new WPI_VictorSPX(8);
    rightDrive2 = new WPI_VictorSPX(9);
    shooterWheel1 = new WPI_VictorSPX(4);
    shooterWheel2 = new WPI_VictorSPX(5);

    rightDrive1.setInverted(true);
    rightDrive2.setInverted(true);

    leftDrive = new MotorControllerGroup(leftDrive1, leftDrive2);
    rightDrive = new MotorControllerGroup(rightDrive1, rightDrive2);

    differentialDrive = new DifferentialDrive(leftDrive, rightDrive);

    servoMotor = new Servo(0);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    double xSpeed = xboxLeft.getRawAxis(1);
    double zRotation = xboxLeft.getRawAxis(4);

    if(Math.abs(xSpeed) >= 0.05 || Math.abs(zRotation) >= 0.05) {
      differentialDrive.arcadeDrive(xSpeed, zRotation);
    } else {
      differentialDrive.arcadeDrive(0, 0);
    }

    if (xboxLeft.getRawButton(1)) {
      shooterWheel1.set(-0.73);
      shooterWheel2.set(0.75);
    } else {
      shooterWheel1.set(0);
      shooterWheel2.set(0);
    }

    if (xboxLeft.getRawButton(2)) {

      // servoMotor.setSpeed(0.1);
      servoMotor.set(0);

    } else if (xboxLeft.getRawButton(3)) {

      // servoMotor.setSpeed(-0.1);
      servoMotor.set(1);

    }

  
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}



  public void turnTowardsBall() {
    //top left is 0, 0
    //bottom right is 320, 240
    // goal is x = 160

    int x = (int) SmartDashboard.getNumber("DB/String 0", 160);

    if (x > 165) {
      // turn right
    } else if (x < 155) {
      // turn left
    }

  }
}
