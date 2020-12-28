//Render Constants
var ACT_OBJ_COLOR = '#FF2D00';
var DES_OBJ_COLOR = '#004CFF';
var MODULE_NOM_RADIUS_PX = 75;
var WHEEL_MAX_SPEED_RPM = 1000;

var swerveStateSigNameList = [ 
    "DtModuleFLAzmthPosAct", //0
    "DtModuleFLAzmthPosDes",
    "DtModuleFLWheelSpdAct",
    "DtModuleFLWheelSpdDes",
    "DtModuleFRAzmthPosAct",
    "DtModuleFRAzmthPosDes",
    "DtModuleFRWheelSpdAct",
    "DtModuleFRWheelSpdDes",
    "DtModuleBLAzmthPosAct",
    "DtModuleBLAzmthPosDes",
    "DtModuleBLWheelSpdAct",
    "DtModuleBLWheelSpdDes",
    "DtModuleBRAzmthPosAct",
    "DtModuleBRAzmthPosDes",
    "DtModuleBRWheelSpdAct",
    "DtModuleBRWheelSpdDes"]; //15

var swerveStateSigIdList = [];
var swerveStateSigValList = [];

var swerveStateInitFinished = false;


function swerveStateSignalListHandler(data) {

    swerveStateSigIdList = getSignalIDsFromNames(swerveStateSigNameList, data);

    if ( !swerveStateSigIdList.every(elem => elem != null)) {
        // If any were null...
        alert("ERROR from Robot PoseView: Could not find all required signals to display swerve state. Not starting.");
    } else {
        if(swerveStateInitFinished == false){
            //Grab a reference to the canvases
            this.canvas = document.getElementById("swerve_state_canvas");
            this.ctx = this.canvas.getContext("2d");

            this.ctx.canvas.height = 500;
            this.ctx.canvas.width = 500;

            //Fire up a new DAQ for the robot to capture the required signals.
            startNewDAQ("swerveState", swerveStateSigIdList, 50);
            swerveStateInitFinished = true;
        }
    }

}

function swerveStateDataHandler(data) {

    if (data.daq_id == "swerveState") {

        //Grab a reference to the canvases
        this.canvas = document.getElementById("swerve_state_canvas");
        this.ctx = this.canvas.getContext("2d");

        this.ctx.clearRect(0, 0, this.ctx.canvas.width, this.ctx.canvas.height);


        swerveStateSigValList = getSignalValuesFromIDs(swerveStateSigIdList, data);

        //console.log(swerveStateSigValList);
        drawModule(this.ctx, swerveStateSigValList[1],  swerveStateSigValList[3],  100, 100, false); //Front Left
        drawModule(this.ctx, swerveStateSigValList[0],  swerveStateSigValList[2],  100, 100, true);  //Front Left
        drawModule(this.ctx, swerveStateSigValList[5],  swerveStateSigValList[7],  400, 100, false); //Front Right
        drawModule(this.ctx, swerveStateSigValList[4],  swerveStateSigValList[6],  400, 100, true);  //Front Right      
        drawModule(this.ctx, swerveStateSigValList[9],  swerveStateSigValList[11], 100, 400, false); //Back Left
        drawModule(this.ctx, swerveStateSigValList[8],  swerveStateSigValList[10], 100, 400, true);  //Back Left
        drawModule(this.ctx, swerveStateSigValList[13], swerveStateSigValList[15], 400, 400, false); //Back Right
        drawModule(this.ctx, swerveStateSigValList[12], swerveStateSigValList[14], 400, 400, true);  //Back Right
    }

}


drawModule = function (ctx_in, rotation_deg, speed_rpm, x_pos_px, y_pos_px, isActual) {

    //one module

    var color = DES_OBJ_COLOR;
    var radius = MODULE_NOM_RADIUS_PX + 10;

    if(isActual){
        color = ACT_OBJ_COLOR;
        radius = MODULE_NOM_RADIUS_PX - 10;
    }

    //Tweak rotation to match the javascript canvas draw angle convention
    rotation_deg *= -1;

    //Rotate to module reference frame
    ctx_in.translate(x_pos_px, y_pos_px);
    ctx_in.rotate(rotation_deg * Math.PI / 180);

    //Solid filled in red robot is for Actual
    ctx_in.beginPath();
    ctx_in.strokeStyle = color;
    ctx_in.lineWidth = "1";
    ctx_in.arc(0, 0, radius, 0, 2 * Math.PI);
    ctx_in.stroke()

    ctx_in.beginPath()
    ctx_in.strokeStyle = color;
    ctx_in.lineWidth = "3";
    ctx_in.moveTo(0,-radius)
    ctx_in.lineTo(0,-radius+15)
    ctx_in.stroke()

    var arrowLen = Math.abs(speed_rpm / WHEEL_MAX_SPEED_RPM * MODULE_NOM_RADIUS_PX * 0.75);

    var arrowOffset = 5;
    if(isActual)
        arrowOffset = -5;

    var speedIsNegative = (speed_rpm < 0);

    if(speedIsNegative){
        ctx_in.rotate(Math.PI);
        arrowOffset *= -1;
    }

    if(arrowLen > 2)
        canvas_arrow(ctx_in, arrowOffset, 0, arrowOffset, -1.0 * (10 + arrowLen));

    if(speedIsNegative)
        ctx_in.rotate(-Math.PI);


    if(isActual){
        //wheel center marker
        ctx_in.beginPath();
        ctx_in.strokeStyle = color;
        ctx_in.lineWidth = "6";
        ctx_in.moveTo(-15, 0);
        ctx_in.lineTo(15, 0);
        ctx_in.closePath();
        ctx_in.stroke(); 
    }

    //Undo rotation/translation
    ctx_in.rotate(-1 * rotation_deg * Math.PI / 180);
    ctx_in.translate(-x_pos_px, -y_pos_px);

}

function canvas_arrow(context, fromx, fromy, tox, toy) {
    var headlen = 10; // length of head in pixels
    var dx = tox - fromx;
    var dy = toy - fromy;
    var angle = Math.atan2(dy, dx);
    context.moveTo(fromx, fromy);
    context.lineTo(tox, toy);
    context.lineTo(tox - headlen * Math.cos(angle - Math.PI / 6), toy - headlen * Math.sin(angle - Math.PI / 6));
    context.moveTo(tox, toy);
    context.lineTo(tox - headlen * Math.cos(angle + Math.PI / 6), toy - headlen * Math.sin(angle + Math.PI / 6));
    context.stroke();
}



registerOpenCallback(requestSignalList);
registerDAQUpdateCallback(swerveStateDataHandler);
registerSignalListCallback(swerveStateSignalListHandler);

