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
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Driving Controls
  XboxController driver;
  XboxController manipulator;

  // DriveTrain Motors
  WPI_VictorSPX leftDrive1;
  WPI_VictorSPX leftDrive2;
  WPI_VictorSPX rightDrive1;
  WPI_VictorSPX rightDrive2;
  MotorControllerGroup leftDrive;
  MotorControllerGroup rightDrive;
  DifferentialDrive differentialDrive;

  WPI_VictorSPX shooterWheel1;
  WPI_VictorSPX shooterWheel2;
  MotorControllerGroup shooter;

  WPI_VictorSPX intake;

  //Servo with a CAM to push the cargo through the shooter motors
  Servo cargoPusher;

  //Used for vision processing
  UsbCamera camera = CameraServer.startAutomaticCapture();
  CvSink cvSink;
  CvSource outputStream;
  Mat source;
  Mat hsvmat;
  Mat coreOutput;

  double visionXLocation = 0;
  double visionYLocation = 0;

  final int CAMERA_WIDTH = 320;
  final int CAMERA_HEIGHT = 240;

  ShooterState shooterState = ShooterState.Halt;
  Timer timer = new Timer();

  /** This function is run when the robot is first started up. */
  @Override
  public void robotInit() {
    cvSink = CameraServer.getVideo();
    outputStream = CameraServer.putVideo("vision", CAMERA_WIDTH, CAMERA_HEIGHT);
    source = new Mat();
    hsvmat = new Mat();
    coreOutput = new Mat();

    driver = new XboxController(0);
    // manipulator = new XboxController(1);

    leftDrive1 = new WPI_VictorSPX(6);
    leftDrive2 = new WPI_VictorSPX(7);
    rightDrive1 = new WPI_VictorSPX(8);
    rightDrive2 = new WPI_VictorSPX(9);
    shooterWheel1 = new WPI_VictorSPX(4);
    shooterWheel2 = new WPI_VictorSPX(5);
    intake = new WPI_VictorSPX(1);

    shooterWheel1.setInverted(true);
    shooter = new MotorControllerGroup(shooterWheel1, shooterWheel2);

    rightDrive1.setInverted(true);
    rightDrive2.setInverted(true);

    leftDrive = new MotorControllerGroup(leftDrive1, leftDrive2);
    rightDrive = new MotorControllerGroup(rightDrive1, rightDrive2);

    differentialDrive = new DifferentialDrive(leftDrive, rightDrive);

    cargoPusher = new Servo(0);
    camera.setResolution(CAMERA_WIDTH, CAMERA_HEIGHT);
  }

  /** This function is called periodically while the robot is on. */
  @Override
  public void robotPeriodic() {
    nextVisionFrame();
  }

  // This function is called periodically to do vision processing computations
  public void nextVisionFrame() {

    // Grab the current camera frame and put it in the source Mat (short for Matrix)
    long valid = cvSink.grabFrame(source);

    // If the camera isn't providing video feed, do not proceed with processing
    if(valid == 0) {
      return;
    }

    // Convert the source image to the HSV (hue, saturation, value) color scheme and place it in hsvmat
    Imgproc.cvtColor(source, hsvmat, Imgproc.COLOR_RGB2HSV_FULL);

    // Highlight every pixel that matches the ball color and place the new matrix in coreOutput
    Core.inRange(hsvmat, new Scalar(120,Integer.MIN_VALUE,Integer.MIN_VALUE), new Scalar(140,Integer.MAX_VALUE,Integer.MAX_VALUE), coreOutput);
    
    // Create an array that will contain every "contour" - a cluster of highlighted pixels
    List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

    // Use Image Processing to detect every contour
    Imgproc.findContours(coreOutput, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

    // Out of all of the detected contours, select the largest one
    double maxArea = 0;
    MatOfPoint maxContour = new MatOfPoint();
    Iterator<MatOfPoint> each = contours.iterator();
    while(each.hasNext()) {
      MatOfPoint wrapper = each.next();
      double area = Imgproc.contourArea(wrapper);
      if(area > maxArea) {
        maxArea = area;
        maxContour = wrapper;
      }
    }

    // For as long as the largest area isn't 0 (i.e., there is at least one contour), record its position
    if(maxArea != 0) {
      Moments m = Imgproc.moments(maxContour, false);
      int x = (int) (m.get_m10() / m.get_m00());
      int y = (int) (m.get_m01() / m.get_m00());

      // Draw a circle where the object is located
      Imgproc.circle(source, new Point(x, y), 10, new Scalar(255, 0, 0));

      // Scales the location to -1.0 to 1.0
      visionXLocation = ((double) x) * 2.0 / CAMERA_WIDTH - 1.0;
      visionYLocation = ((double) y) * 2.0 / CAMERA_HEIGHT - 1.0;
    }

    // Output the camera feed to the SmartDashboard
    outputStream.putFrame(source);
  }

  /** This function is called once when autonomous is enabled. */
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
    // Get the drive speeds from the driver XBox controller
    double xSpeed = driver.getRawAxis(1);
    double zRotation = driver.getRawAxis(4);

    // If the drive joysticks are off-center, allow the driver to move the robot
    if(Math.abs(xSpeed) >= 0.05 || Math.abs(zRotation) >= 0.05) {
      differentialDrive.arcadeDrive(xSpeed, zRotation);
    } else {
      differentialDrive.arcadeDrive(0, 0);
    }

    if(driver.getRawButton(4)) {
      intake.set(0.25);
    } else {
      intake.set(0);
    }
    
    switch(shooterState) {
      case WarmUpHigh:
        shooter.set(0.85);
        cargoPusher.set(0);
        if(timer.get() >= 3.0) {
          shooterState = ShooterState.CargoUp;
          resetTimer();
        }
      break;
      case WarmUpLow:
        shooter.set(0.4);
        cargoPusher.set(0);
        if(timer.get() >= 3.0) {
          shooterState = ShooterState.CargoUp;
          resetTimer();
        }
      break;
      case CargoUp:
        shooter.set(shooter.get());
        cargoPusher.set(1.0);
        if(timer.get() >= 1.5) {
          shooterState = ShooterState.Halt;
          shooter.set(0);
          cargoPusher.set(0);
        }
      break;
      case Halt:
        shooter.set(driver.getRawButton(1) ? 0.85 : 0);
        if(driver.getRawButton(2)) {
          cargoPusher.set(0);
        } else if(driver.getRawButton(3)) {
          cargoPusher.set(1);
        }
        if(driver.getRawButtonPressed(8)) {
          shooterState = ShooterState.WarmUpHigh;
          resetTimer();
        } else if(driver.getRawButtonPressed(7)) {
          shooterState = ShooterState.WarmUpLow;
          resetTimer();
        }
      break;
      default:
      break;
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

  void resetTimer() {
    timer.stop();
    timer.reset();
    timer.start();
  }

  public enum ShooterState {
    WarmUpHigh, WarmUpLow, CargoUp, Halt
  };
}
