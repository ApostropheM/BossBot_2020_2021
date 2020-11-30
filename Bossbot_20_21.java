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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
@TeleOp

public class Bossbot_20_21 extends LinearOpMode {
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    
    private DcMotor pivot;
    private DcMotor launch;
    
    
    private CRServo intake;
    private Servo lock;
    

    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        
        pivot = hardwareMap.get(DcMotor.class, "pivot");
        launch = hardwareMap.get(DcMotor.class, "launch");
        
        
        intake = hardwareMap.get(CRServo.class, "intake");
        lock = hardwareMap.get(Servo.class, "lock");
        
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // mecanum drive
            drive(gamepad1.right_trigger - gamepad1.left_trigger, gamepad1.left_stick_x, gamepad1.right_stick_x);
            
            // launch
            if (gamepad2.x) {
                launch.setPower(1);
            } else {
                launch.setPower(0);
            }
            
            // pivot
            if (gamepad2.dpad_up) {
                pivot.setPower(0.25);
            } else if (gamepad2.dpad_down) {
                pivot.setPower(-0.25);
            } else {
                pivot.setPower(0);
                pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                
            }
            
            // intake
            if (gamepad2.right_bumper) {
                intake.setPower(1);
            } else if (gamepad2.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
            
            // lock
            if (gamepad2.a) {
                lock.setPosition(1);
            } else {
                lock.setPosition(-1);
            }
            
            
            
        }
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
