/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Martian TeleOp", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class MartianTeleOp extends LinearOpMode {
    //private Gyroscope imu;
    private DcMotor motorRearLeft, motorRearRight, motorFrontLeft, motorFrontRight, motorArm;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange;
    private ColorSensor sensorColor;
    private Servo servoLeftClaw, servoRightClaw;
    public Servo    sensorArm   = null;
    private double dblSpeedMultiplier = 0.5;  
    private double dblTurnSpeed = 0.5;


    @Override
    public void runOpMode() {
        
        motorRearRight = hardwareMap.get(DcMotor.class, "motorRearRight");
        motorRearLeft = hardwareMap.get(DcMotor.class, "motorRearLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        
        //motorArm = hardwareMap.get(DcMotor.class, "motorArm");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        //sensorColor = hardwareMap.get(ColorSensor.class, "sensorColorRange");
    //    servoLeftClaw = hardwareMap.get(Servo.class, "servoLeftClaw");
      //  servoRightClaw = hardwareMap.get(Servo.class, "servoRightClaw");
    //    sensorArm = hardwareMap.get(Servo.class, "servoSensorArm");

        // Set digital channel to input Mode
      //  digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        
        //telemetry.addData("Status", "Initialized");
        //telemetry.update();
        
        //sensorArm.setPosition(0.50);
            
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        
        
        
        
        double tgtPowerRearLeft = 0;
        double tgtPowerRearRight = 0;
        double tgtPowerFrontLeft = 0;
        double tgtPowerFrontRight = 0;
        
        double tgtPowerRightStickX = 0;
        
        
        boolean tgtPowerRightBumper = false;
        boolean tgtPowerLeftBumper = false;
        
        double tgtPowerArm = 0;
        

        while (opModeIsActive()) {
            
            // Set the controls for Gamepad 1
            tgtPowerFrontLeft = this.gamepad1.left_stick_y;
            tgtPowerRearLeft = this.gamepad1.left_stick_y;
            tgtPowerFrontRight = -this.gamepad1.left_stick_y;
            tgtPowerRearRight = -this.gamepad1.left_stick_y;
            
            //tgtPowerFrontLeft = this.gamepad1.right_stick_x;
            //tgtPowerRearLeft = this.gamepad1.right_stick_x;
            
            //tgtPowerFrontRight = -this.gamepad1.right_stick_x;
            //tgtPowerRearRight = -this.gamepad1.right_stick_x;
            
            tgtPowerRightStickX = this.gamepad1.right_stick_x;
            
            
            tgtPowerRightBumper = this.gamepad1.right_bumper;
            tgtPowerLeftBumper = this.gamepad1.left_bumper;
            
            // Set Controls for Gamepad 1
            //if (gamepad1.a) {
            //    dblSpeedMultiplier = 0.6;
            //} else if (gamepad1.b) {
            //    dblSpeedMultiplier = 1.0;
            //}
            
            // Check for right or left bumber and Straf if pressed
            
            if (tgtPowerLeftBumper) {
                motorRearLeft.setPower(-1);
                motorRearRight.setPower(-1);
                motorFrontLeft.setPower(1);
                motorFrontRight.setPower(1);
            }
            else if (tgtPowerRightBumper) {
                motorRearLeft.setPower(1);
                motorRearRight.setPower(1);
                motorFrontLeft.setPower(-1);
                motorFrontRight.setPower(-1);
            }
            else if (tgtPowerRightStickX != 0) {
                
                motorRearLeft.setPower(tgtPowerRightStickX*-dblTurnSpeed);
                motorFrontLeft.setPower(tgtPowerRightStickX*-dblTurnSpeed);
                motorRearRight.setPower(tgtPowerRightStickX*-dblTurnSpeed);
                motorFrontRight.setPower(tgtPowerRightStickX*-dblTurnSpeed);
            }
          
            else {
                motorRearLeft.setPower(tgtPowerRearLeft*dblSpeedMultiplier);
                motorRearRight.setPower(tgtPowerRearRight*dblSpeedMultiplier);
                motorFrontLeft.setPower(tgtPowerFrontLeft*dblSpeedMultiplier);
                motorFrontRight.setPower(tgtPowerFrontRight*dblSpeedMultiplier);
            } 
            
            telemetry.addData("Front Left Target Power", tgtPowerFrontLeft);
            //telemetry.addData("Motor Power", motorFrontLeft.getPower());
            
            telemetry.addData("Rear Left Target Power", tgtPowerRearLeft);
            //telemetry.addData("Motor Power", motorRearLeft.getPower());
            
            telemetry.addData("Front Right Target Power", tgtPowerFrontRight);
            //telemetry.addData("Motor Power", motorFrontRight.getPower());
            
            telemetry.addData("Rear Right Target Power", tgtPowerRearRight);
            //telemetry.addData("Motor Power", motorRearRight.getPower());
            
            //tgtPowerRearRight = -this.gamepad1.right_trigger;
            //motorRearLeft.setPower(tgtPowerRearLeft);
            
            //tgtPowerRearLeft = this.gamepad1.right_trigger;
            //motorRearRight.setPower(tgtPowerRearRight);
          
            // Set the controls for Gamepad 2
            //tgtPowerArm = this.gamepad2.right_stick_y;
            //motorArm.setPower(tgtPowerArm);
            //motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            
            // Check to see if you need to move Servo
            //if (gamepad2.a) {
            //    //move to 0 degree
            //    servoLeftClaw.setPosition(0.2);
            //    servoRightClaw.setPosition(0.8);
        
            //} else if (gamepad2.b) {
            //    //move to 180 degrees
            //    servoLeftClaw.setPosition(1);
            //    servoRightClaw.setPosition(0);
            //}
            
            // digitalTouch - Check if the button was pressed
            //if (digitalTouch.getState() == false) {
            //    // Button is pressed
            //    telemetry.addData("Button", "PRESSED");
            //} else {
            //    // Button is not pressed
            //    telemetry.addData("Button", "NOT PRESSED");
            //}
            
            // Color Sensor
            //if (sensorColor.red() < 25 && sensorColor.blue() < 25 ) {
            //    telemetry.addData("Color", "UNKNOWN");
            //} else if (sensorColor.red() < sensorColor.blue() ) {
            //    telemetry.addData("Color", "BLUE");
            //} else {
            //    telemetry.addData("Color", "RED");
            //}
            
            //telemetry.addData("servoLeftClaw", servoLeftClaw.getPosition());
            //telemetry.addData("servoRightClaw", servoRightClaw.getPosition());
            
            //telemetry.addData("Distance (cm)", sensorColorRange.getDistance(DistanceUnit.CM));
            
            //telemetry.addData("Color I2cAdress", sensorColor.getI2cAddress());
            //telemetry.addData("Color Red", sensorColor.red());
            //telemetry.addData("Color Blue", sensorColor.blue());
            
            telemetry.addData("Status", "Running");
            telemetry.update();
        
        }
    }
}
