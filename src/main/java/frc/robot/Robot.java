package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.PWM;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.sim.mockdata.DriverStationDataJNI;

public class Robot extends TimedRobot {

      public PixyI2C power_Cell_Pixy;
      Port port = Port.kOnboard;
      String print;
      public PixyPacket[] packet1 = new PixyPacket[7];

      private Joystick Pilot_Stick, Co_Pilot_Stick1, Co_Pilot_Stick2;

      private CANSparkMax leftMotor, leftFollowMotor, rightMotor, rightFollowMotor;
      private CANSparkMax pickupMotor, magazineMotor, acceleratorMotor;
      private CANSparkMax turretMotor, hoodMotor, shooterMotor, climberMotor;

      private CANEncoder shooterEncoder;
      private CANPIDController shooterController;

      private CANDigitalInput forwardLimit_hood;

      private static final SPI.Port kGyroPort = SPI.Port.kOnboardCS0;

      private CANPIDController m_pidController_left, m_pidController_right;

      private static ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(kGyroPort);

      private AnalogInput Pixy_Forward_Input, Pixy_Turret;

      private PWM LightStrip;

      private Compressor airCompressor;
      private DoubleSolenoid gearShift, pickupShift;

      private CANEncoder m_encoder_left, m_encoder_right;

      private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, MaxRPM;

      private Double X, Y, invX, V, W, R, L, turret_speed, DriveAdjust, magazine_speed;

      private int WWW, DDD, Last_Value;
      private int Target_X_Avg, Target_Y_Avg, Turret_Start_Position;;

      private int counter, hoodPosition, shooterSpeed;

      private boolean hoodZero, startUp;

      private Timer timer;

      private CANEncoder turret_Encoder;
      private CANPIDController turret_Controller_Move;

      private CANEncoder hood_Encoder;
      private CANPIDController hood_Controller;

      private PIDController Pixy_Turret_Controller;

      private PIDController follow_Ball_Controller, gyro_Controller;

      @Override
      public void robotInit() {

            hoodPosition = -17;
            shooterSpeed = 5500;
            hoodZero = false;
            startUp = true;
            DriveAdjust = 0.0;

            counter = 0;

            Pilot_Stick = new Joystick(Parameters.USB_STICK_PILOT);
            Co_Pilot_Stick1 = new Joystick(Parameters.USB_STICK_COPILOT1);
            Co_Pilot_Stick2 = new Joystick(Parameters.USB_STICK_COPILOT2);

            CameraServer.getInstance().startAutomaticCapture();

            // power_Cell_Pixy = new PixyI2C("Power Cell", new I2C(port, 0x54), packet1, new
            // PixyException(print),
            // new PixyPacket());

            /**********************************************************************************/
            m_gyro.calibrate();

            leftMotor = new CANSparkMax(Parameters.CANIDs.DRIVE_LEFT_LEADER.getid(), MotorType.kBrushless);
            leftFollowMotor = new CANSparkMax(Parameters.CANIDs.DRIVE_LEFT_FOLLOWER.getid(), MotorType.kBrushless);
            rightMotor = new CANSparkMax(Parameters.CANIDs.DRIVE_RIGHT_LEADER.getid(), MotorType.kBrushless);
            rightFollowMotor = new CANSparkMax(Parameters.CANIDs.DRIVE_RIGHT_FOLLOWER.getid(), MotorType.kBrushless);

            leftMotor.restoreFactoryDefaults();
            leftFollowMotor.restoreFactoryDefaults();
            rightMotor.restoreFactoryDefaults();
            rightFollowMotor.restoreFactoryDefaults();

            m_pidController_left = leftMotor.getPIDController();
            m_pidController_right = rightMotor.getPIDController();

            m_encoder_left = leftMotor.getEncoder();
            m_encoder_right = rightMotor.getEncoder();

            leftFollowMotor.follow(leftMotor);
            rightFollowMotor.follow(rightMotor);

            pickupMotor = new CANSparkMax(Parameters.CANIDs.PICKUP.getid(), MotorType.kBrushless);
            pickupMotor.restoreFactoryDefaults();

            magazineMotor = new CANSparkMax(Parameters.CANIDs.MAGAZINE.getid(), MotorType.kBrushless);
            magazineMotor.restoreFactoryDefaults();

            acceleratorMotor = new CANSparkMax(Parameters.CANIDs.ACCELERATOR.getid(), MotorType.kBrushless);
            acceleratorMotor.restoreFactoryDefaults();

            turretMotor = new CANSparkMax(Parameters.CANIDs.TURRET_DIRECTION.getid(), MotorType.kBrushless);
            turretMotor.restoreFactoryDefaults();

            turretMotor.setSoftLimit(SoftLimitDirection.kForward, 300.0f); // was 150
            turretMotor.setSoftLimit(SoftLimitDirection.kReverse, -300.0f); // was -150
            turretMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
            turretMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

            hoodMotor = new CANSparkMax(Parameters.CANIDs.TURRET_HOOD.getid(), MotorType.kBrushless);
            hoodMotor.restoreFactoryDefaults();
            hood_Encoder = hoodMotor.getEncoder();
            hood_Controller = hoodMotor.getPIDController();

            kP = 0.025;
            kI = 1e-5;
            kD = 0;
            kIz = 0;
            kFF = 0;
            kMaxOutput = 1;
            kMinOutput = -1;

            hood_Controller.setP(0.03);
            hood_Controller.setI(1e-5);
            hood_Controller.setD(kD);
            hood_Controller.setIZone(kIz);
            hood_Controller.setFF(kFF);
            hood_Controller.setOutputRange(kMinOutput, kMaxOutput);

            shooterMotor = new CANSparkMax(32, MotorType.kBrushless);
            shooterMotor.restoreFactoryDefaults();

            shooterMotor.setInverted(true);

            shooterEncoder = shooterMotor.getEncoder();

            shooterController = shooterMotor.getPIDController();
            shooterController.setP(Constants.kP_Shooter);
            shooterController.setI(Constants.kI_Shooter);
            shooterController.setD(Constants.kD_Shooter);
            shooterController.setIZone(Constants.klz_Shooter);
            shooterController.setFF(Constants.kFF_Shooter);
            shooterController.setOutputRange(Constants.kMinOutput_Shooter, Constants.kMaxOutput_Shooter);

            climberMotor = new CANSparkMax(60, MotorType.kBrushless);
            climberMotor.restoreFactoryDefaults();

            forwardLimit_hood = hoodMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
            forwardLimit_hood.enableLimitSwitch(true);

            airCompressor = new Compressor(0);
            gearShift = new DoubleSolenoid(0, 1);
            pickupShift = new DoubleSolenoid(2, 3);

            Pixy_Turret = new AnalogInput(2);
            Pixy_Forward_Input = new AnalogInput(3);
            /**********************************************************************************/

            // LightStrip = new PWM(0);

            follow_Ball_Controller = new PIDController(Constants.kP_Drive_Pixy, Constants.kI_Drive_Pixy,
                        Constants.kD_Drive_Pixy); // Yellow Ball
            gyro_Controller = new PIDController(Constants.kP_Drive_Gyro, Constants.kI_Drive_Gyro,
                        Constants.kD_Drive_Gyro); // Gyro

            Pixy_Turret_Controller = new PIDController(Constants.kP_Turret_Camera, Constants.kI_Turret_Camera,
                        Constants.kD_Turret_Camera);

            turret_Encoder = turretMotor.getEncoder();

            Target_X_Avg = 160;

            m_gyro.reset();

            timer = new Timer();

      }

      @Override
      public void teleopPeriodic() {
            zeroHoodPosition();

            // for (int i = 0; i < packet1.length; i++) {
            // packet1[i] = null;
            // }

            // for (int i = 1; i < 8; i++) {
            // try {
            // // read packet for signature i (counting from 1, see pixy docs)
            // packet1[i - 1] = power_Cell_Pixy.readPacket(i);
            // } catch (PixyException e) {
            // // SmartDashboard.putString("Power Cell Pixy Error: " + i, "exception");
            // }
            // if (packet1[i - 1] == null) {
            // // SmartDashboard.putString("Power Cell Pixy Error: " + i, "True");
            // continue;
            // }
            // // SmartDashboard.putNumber("Power Cell Pixy X Value: ", packet1[i - 1].X);
            // // SmartDashboard.putNumber("Power Cell Pixy Y Value: ", packet1[i - 1].Y);
            // // SmartDashboard.putNumber("Power Cell Pixy Width Value: ", packet1[i -
            // 1].Width);
            // // SmartDashboard.putNumber("Power Cell Pixy Height Value: ", packet1[i -
            // 1].Height);
            // // SmartDashboard.putString("Power Cell Pixy Error: " + i, "False");

            // WWW = packet1[i - 1].X;
            // // if (Math.abs(WWW-Last_Value)<3)
            // if (Math.abs(WWW - 160) < 75) // && (Target_Y_Avg > 50)) Fixme
            // Target_X_Avg = (int) (Target_X_Avg + WWW) / 2;

            // DDD = packet1[i - 1].Y;
            // Target_Y_Avg = (int) (Target_Y_Avg + DDD) / 2;

            // Last_Value = packet1[i - 1].X;

            // // SmartDashboard.putNumber("Power Cell Pixy X Value: ", Target_X_Avg);

            // }

            Drive();

            if (Pilot_Stick.getRawButton(Parameters.PILOT_BUTTON_1)) {
                  gearShift.set(Value.kReverse);
            } else {
                  gearShift.set(Value.kForward);
            }
            rightMotor.set(R - DriveAdjust);
            leftMotor.set(L - DriveAdjust);

            // Pickup Commands
            if (Co_Pilot_Stick1.getRawButton(Parameters.COPILOT1_PICKUP) == true) {
                  pickupMotor.set(0.5);
                  pickupShift.set(Value.kForward);

            } else {
                  if (Co_Pilot_Stick2.getRawButton(Parameters.COPILOT2_PICKUP_REV) == true) {
                        pickupShift.set(Value.kForward);
                        pickupMotor.set(-0.5);
                  } else {
                        pickupMotor.set(0.0);
                        pickupShift.set(Value.kReverse);
                  }
            }

            // Magazine Commands

            if (Co_Pilot_Stick2.getRawButton(Parameters.COPILOT2_CLIMB) == true) {
                  magazine_speed = 0.4;
            } else {
                  magazine_speed = 0.0;
            }
            if (Co_Pilot_Stick1.getRawButton(Parameters.COPILOT1_MAGAZINE_DOWN) == true) {
                  magazineMotor.set(0.5 + magazine_speed);
            } else {
                  if (Co_Pilot_Stick2.getRawButton(Parameters.COPILOT2_MAGAZINE_UP) == true) {
                        magazineMotor.set(-0.3 - magazine_speed);
                  } else {
                        magazineMotor.set(0.0);
                  }
            }
            // Accerator Command

            // if (Co_Pilot_Stick2.getRawButton(Parameters.COPILOT2_ACCEL_FWD) == true) {
            // acceleratorMotor.set(-0.7);
            // } else {
            // if (Co_Pilot_Stick2.getRawButton(Parameters.COPILOT2_ACELL_REV) == true) {
            // acceleratorMotor.set(0.7);
            // } else
            // { acceleratorMotor.set(0.0); }
            // }

            // Turret Commands
            if (Co_Pilot_Stick1.getRawButton(Parameters.COPILOT1_TURRET_FINE_ADJUST) == true) {
                  turret_speed = 0.03;
            } else {
                  turret_speed = 0.25;
            }

            if (Co_Pilot_Stick1.getRawButton(Parameters.COPILOT1_JOYSTICK_TURRET_LEFT) == true) {
                  turretMotor.set(turret_speed);
            } else {
                  if (Co_Pilot_Stick1.getRawButton(Parameters.COPILOT1_JOYSTICK_TURRET_RIGHT) == true) {
                        turretMotor.set(-turret_speed);
                  } else
                        turretMotor.set(0.0);
            }

            if (Co_Pilot_Stick1.getRawButton(Parameters.COPILOT1_FIND_TARGET) == true) {
                  double drivePower = Pixy_Turret_Controller.calculate(Pixy_Turret.getAverageVoltage(),
                              (3.3 / 2.0d) + .1); // Fix me 139????
                  // SmartDashboard.putNumber("Turret Pixy", Pixy_Turret.getAverageVoltage());
                  // System.out.println(drivePower);
                  turretMotor.set(drivePower);
            }
            // else
            // if (Math.abs(drivePower) < .1)
            // turretMotor.set(0.0);

            // Shooter Coommands

            if (Co_Pilot_Stick1.getRawButton(Parameters.COPILOT1_SHOOT) == true) {
                  shooterController.setReference(shooterSpeed, ControlType.kVelocity);
                  acceleratorMotor.set(-0.7);
            } else {
                  shooterController.setReference(0, ControlType.kVelocity);
                  acceleratorMotor.set(0.0);
            }

            // Hood Commands

            // SmartDashboard.putNumber("hood position", hood_Encoder.getPosition());

            if (!startUp) {
                  if (Co_Pilot_Stick1.getRawButton(Parameters.COPILOT1_HOOD_CLOSE) == true) {
                        hoodPosition = -17;
                        shooterSpeed = 5500;
                  }
                  if (Co_Pilot_Stick2.getRawButton(Parameters.COPILOT2_HOOD_MEDIUM) == true) {
                        hoodPosition = -20;
                        shooterSpeed = 5500;
                  }
                  if (Co_Pilot_Stick1.getRawButton(Parameters.COPILOT1_HOOD_FAR) == true) {
                        hoodPosition = -23;
                        shooterSpeed = 5500;
                  }
                  hood_Controller.setReference(hoodPosition, ControlType.kPosition);
            }
            if (Pilot_Stick.getRawButton(Parameters.PILOT_BUTTON_2) == true) {
                  double Pixy_Avg_Volt = Pixy_Forward_Input.getAverageVoltage();
                  if ((Pixy_Avg_Volt > .15) && (Pixy_Avg_Volt < 3.0)) {
                        DriveAdjust = follow_Ball_Controller.calculate(Pixy_Avg_Volt, 3.3 / 2.0);
                  } else {
                        DriveAdjust = 0.0;
                  }
            } else {
                  DriveAdjust = 0.0;
            }

            // Climber Commands

            if (Co_Pilot_Stick2.getRawButton(8) == true) { // Green Button
                  climberMotor.set(0.4);
            } else {
                  climberMotor.set(0.0);
            }

      }

      public void Drive() {

            // Y = Pilot_Stick.getX(GenericHID.Hand.kLeft);
            Y = Pilot_Stick.getX();
            // X = Pilot_Stick.getY(GenericHID.Hand.kLeft);
            X = Pilot_Stick.getY();

            if (Math.abs(X) < .15)
                  X = 0.0;
            if (Math.abs(Y) < .15)
                  Y = 0.0;

            // X = X;
            V = ((1.0 - Math.abs(X)) * Y) + Y;
            W = ((1.0 - Math.abs(Y)) * X) + X;
            R = (V + W) / 3; // / 4;
            L = (V - W) / 3; // / 4;

      }

      @Override
      public void autonomousInit() {
            hoodPosition = -14;
            shooterSpeed = 5500;
            hoodZero = false;
            startUp = true;
      }

      /**
       * This function is called periodically during autonomous.
       */
      @Override
      public void autonomousPeriodic() {
            zeroHoodPosition();

            if (!startUp) {
            // Figure out what autonomous code to run and invoke it
            autonomousShootBackupTurnAround();
            }
      }

      private void zeroHoodPosition() {
            // Start up Begin

            /**
             * 
             */
            if (startUp) {

                  if (forwardLimit_hood.get() == false) {
                        hoodMotor.set(0.2);
                  } else {
                        hoodMotor.set(0.0);
                        hood_Encoder.setPosition(0);
                        turret_Encoder.setPosition(0.0); /// FIXME
                        hoodMotor.setSoftLimit(SoftLimitDirection.kForward, 0.0f);
                        hoodMotor.setSoftLimit(SoftLimitDirection.kReverse, -43.0f);
                        hoodMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
                        hoodMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
                        startUp = false;
                        hoodZero = true;
                  }
            }
            // Start up End
      }

      private void autonomousShootBackupTurnAround() {
            double matchTime = 15.0 - DriverStation.getInstance().getMatchTime();

            if (matchTime <= 2.0) {
                  // Turn on shooter and wait for it to come up to speed for 2 seconds
                  hood_Controller.setReference(hoodPosition, ControlType.kPosition);
                  shooterController.setReference(shooterSpeed, ControlType.kVelocity);
            } else if (matchTime <= 7.0) {
                  // Shoot for 5 seconds
                  acceleratorMotor.set(-0.7);
                  magazineMotor.set(0.8);
            } else if (matchTime <= 8.5) {
                  // Backup for 1.5 seconds
                  acceleratorMotor.set(0.0); // Shut down shooting
                  magazineMotor.set(0.0);
                  shooterController.setReference(0, ControlType.kVelocity);
                  rightMotor.set(.3); // Back up
                  leftMotor.set(-.3); //
            } else if (matchTime <= 10.7) {
                  // Turn around approx. 180
                  rightMotor.set(.2); // Tu;rn ~ 180
                  leftMotor.set(.2); //
            } else if (matchTime <= 11.00) {
                  // All stop
                  rightMotor.set(0.0); // All stop
                  leftMotor.set(0.0); //
                  // Turn on magazine
                  // Deploy & turn on Pickup
            } else if (matchTime <= 14.0) {
                  // Follow ball
                  DriveAdjust = follow_Ball_Controller.calculate(Pixy_Forward_Input.getAverageVoltage(), 3.3 / 2.0);
                  rightMotor.set(-.2 - DriveAdjust); // Drive forward
                  leftMotor.set(.2 - DriveAdjust); //
            } else {
                  // All stop
                  rightMotor.set(0.0); // All stop
                  leftMotor.set(0.0); //
            }
      }
}