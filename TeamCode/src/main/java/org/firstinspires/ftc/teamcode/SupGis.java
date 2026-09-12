package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Disabled
@Autonomous(name="testHello")
public class SupGis extends OpMode {

    @Override//*** override the init() method extended from OpMode
    public void init(){
       telemetry.addData("Sup","GISTIC");
       //*** using addData() Method to let the message pop out at the Driver hub(DS/DH)

    }

    @Override//Same as the upper one
    public void loop(){

    }
}
