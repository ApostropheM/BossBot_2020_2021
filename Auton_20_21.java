/*
Copyright 2018 FIRST Tech Challenge Team 524

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Autonomous
public class Auton_20_21 extends LinearOpMode {
    private BNO055IMU imu;
    
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    
    private DcMotor pivot;
    private DcMotor launch;
    
    
    private CRServo intake;
    private Servo lock;
    
    private int numRings = 4;
    private DigitalChannel top_ir;
    private DigitalChannel bot_ir;
    

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        
        pivot = hardwareMap.get(DcMotor.class, "pivot");
        launch = hardwareMap.get(DcMotor.class, "launch");
        
        
        intake = hardwareMap.get(CRServo.class, "intake");
        lock = hardwareMap.get(Servo.class, "lock");
        
        top_ir = hardwareMap.get(DigitalChannel.class, "top_ir");
        bot_ir = hardwareMap.get(DigitalChannel.class, "bot_ir");
        
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        
        com.qualcomm.hardware.bosch.BNO055IMU.Parameters imuParameters;
        double angles;
        boolean i = false;
        double start;
        double init_angle;
    
        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            
            lock.setPosition(1);
            fd(0.25,0,1);
            drift(0.5, 500);
            fd(0.25,0,0.75);
            numRings = countRings();
            telemetry.addData("rings", numRings);
            telemetry.update();
            drift(0.5, 500);
            yaw(0.25, -10);
            launch.setPower(1);
            sleep(2000);
            intake.setPower(1);
            sleep(5000);
            intake.setPower(0);
            launch.setPower(0);
            
            fd(0.75, 0,0.5);
            
            
            if (numRings == 0) {
                fd(0.5, 0, 0.5);
                drift(-0.5, 750);
                pivot.setPower(0.5);
                sleep(500);
                pivot.setPower(0);
                lock.setPosition(-1);
            } else if (numRings == 1) {
                fd(0.5, 0, 1);
                drift(-0.5, 250);
                pivot.setPower(0.5);
                sleep(500);
                pivot.setPower(0);
                lock.setPosition(-1);
                sleep(2000);
                drift(0.25, 1000);
                fd(-0.75, 0, 0.5);
            } else {
                fd(0.5, 0, 1.75);
                drift(-0.5, 750);
                pivot.setPower(0.5);
                sleep(500);
                pivot.setPower(0);
                lock.setPosition(-1);
                sleep(2000);
                drift(0.25, 1000);
                fd(-0.75, 0, 1);
            }
            
            
        }
    }
    
    
    private int countRings() {
        if (top_ir.getState() && bot_ir.getState()) {
            return 0;
        } else if (top_ir.getState() && !bot_ir.getState()) {
            return 1;
        } else {
            return 4;
        }
    }
    
    
    private void fd(double power, double angle, double interval) {
        double start_time = getRuntime();
        double curr_angle;
        while (getRuntime() - start_time < interval) {
          curr_angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
          if (curr_angle-angle > 5) {
            drive(power,power/2,0);
          } else if (curr_angle-angle < -5) {
            drive(power,-power/2,0);
          } else {
            drive(power,0,0);
          }
        }
        drive(-power/4,0,0);
        sleep(250);
        brake(250);
    }
      
    private void yaw(double power, double angle) {
        double curr_angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (angle!=180){
          while (Math.abs(curr_angle-angle)>3) {
            curr_angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            drive(0,power,0);
          }
        } else {
          while (Math.abs(angle-Math.abs(curr_angle))>3) {
            curr_angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            drive(0,power,0);
          }
        }
        brake(250);
    }
      
    private void drift(double power, long dur) {
        drive(0,0,power);
        sleep(dur);
        drive(0,0,-power);
        sleep(250);
        brake(250);
    }
      
    private void brake(long dur) {
        drive(0,0,0);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(dur);
    }
    
    
    
    
    
    private void drive(double fd, double yaw, double drift) {
        if (Math.abs(yaw)<0.15) {
            yaw = 0;
        }
        if (Math.abs(drift)<0.15) {
            drift = 0;
        }
        double p1 = fd + yaw + drift;
        double p2 = fd - yaw - drift;
        double p3 = fd - yaw + drift;
        double p4 = fd + yaw - drift;
        
        if (p1 > 1) {
            p1 = 1;
        }
        else if (p1 < -1) {
            p1 = -1;
        }
        if (p2 > 1) {
            p2 = 1;
        }
        else if (p2 < -1) {
            p2 = -1;
        }
        if (p3 > 1) {
            p3 = 1;
        }
        else if (p3 < -1) {
            p3 = -1;
        }
        if (p4 > 1) {
            p4 = 1;
        }
        else if (p4 < -1) {
            p4 = -1;
        }
        motor1.setPower(p1);
        motor2.setPower(-p2);
        motor3.setPower(-p3);
        motor4.setPower(p4);
    }
}
