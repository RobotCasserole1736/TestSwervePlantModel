//Config - adjust this year to year
var ROBOT_W_FT = 2;
var ROBOT_L_FT = 2.5;
var FIELDPOLY_FT =
    [[0, 0],[54, 0],[54, 27],[0,27],[0, 0]];  


//Render Constants
var PX_PER_FOOT = 15;
var FIELD_COLOR = '#534F4D';
var FIELD_BG_PX_PER_FOOT = 1306.0/27.0;
var BOT_COLOR = '#d22';
var RED_FIELD_ELEMENT_COLOR = '#FF2D00';
var BLUE_FIELD_ELEMENT_COLOR = '#004CFF';
var TAPE_COLOR = '#FFFFFF';
var CANVAS_MARGIN_PX = 20;

var ROBOT_W_PX = 0;
var ROBOT_L_PX = 0;

var desPoseX = 0;
var desPoseY = 0;
var desPoseT = 0;
var actPoseX = 0;
var actPoseY = 0;
var actPoseT = 0;
var estPoseX = 0;
var estPoseY = 0;
var estPoseT = 0;

var botPrevDesPoseX = -1; 
var botPrevDesPoseY = -1;
var botPrevActPoseX = -1;
var botPrevActPoseY = -1;
var botPrevEstPoseX = -1;
var botPrevEstPoseY = -1;

var DRAW_STYLE_ACTUAL = 0;
var DRAW_STYLE_DESIRED = 1;
var DRAW_STYLE_ESTIMATED = 2;

var sigNameList = [ 
    "botDesPoseX",
    "botDesPoseY",
    "botDesPoseT",
    "botActPoseX",
    "botActPoseY",
    "botActPoseT",
    "botEstPoseX",
    "botEstPoseY",
    "botEstPoseT"];

var sigIdList = [];
var sigValList = [];

// In how we draw our field image, WPI convention says 
// positive X motion is toward the top of the screen,
// Positive Y motion is toward the left of the screen.
function actLocToPx(act_x_ft, act_y_ft, origin_px_x, origin_px_y) {
    //Convert the WPI convention to pixels on our screen.
    var px_x = Math.round(origin_px_x - act_y_ft * PX_PER_FOOT);
    var px_y = Math.round(origin_px_y - act_x_ft * PX_PER_FOOT);
    return [px_x, px_y];
}

var poseViewInitCompleted = false;

function poseSignalListHandler(data) {

    //Grab a reference to the canvases
    this.canvas = document.getElementById("field_bg_canvas");
    this.ctx = this.canvas.getContext("2d");

    this.canvas_robot = document.getElementById("robot_canvas");
    this.ctx_robot = this.canvas_robot.getContext("2d");

    this.canvas_path = document.getElementById("path_canvas");
    this.ctx_path = this.canvas_path.getContext("2d");

    sigIdList = getSignalIDsFromNames(sigNameList, data);


    if ( !sigIdList.every(elem => elem != null)) {
        // If any were null...
        alert("ERROR from Robot PoseView: Could not find all required signals to drive robot. Not starting.");
    } else {

        if(poseViewInitCompleted == false){
            //Handle view init information

            //Get extrema of the described shape and set canvas size
            max_x_px = 0;
            min_x_px = 0;
            max_y_px = 0;
            min_y_px = 0;
            for (i = 0; i < FIELDPOLY_FT.length; i++) {
                x_px = FIELDPOLY_FT[i][0] * PX_PER_FOOT;
                y_px = FIELDPOLY_FT[i][1] * PX_PER_FOOT;

                max_x_px = Math.max(x_px, max_x_px);
                min_x_px = Math.min(x_px, min_x_px);
                max_y_px = Math.max(y_px, max_y_px);
                min_y_px = Math.min(y_px, min_y_px);
            }

            //Adjust width/height of everything based on the field dimensions requested.
            const image = document.getElementById('source');
            bg_image_width_px = image.width / FIELD_BG_PX_PER_FOOT * PX_PER_FOOT;
            bg_image_height_px = image.height / FIELD_BG_PX_PER_FOOT * PX_PER_FOOT;
            this.ctx.canvas.height = bg_image_height_px;
            this.ctx.canvas.width = bg_image_width_px;
            this.ctx_robot.canvas.height = this.ctx.canvas.height;
            this.ctx_robot.canvas.width = this.ctx.canvas.width;
            this.ctx_path.canvas.height = this.ctx.canvas.height;
            this.ctx_path.canvas.width = this.ctx.canvas.width;
            document.getElementById("container").style.height = this.ctx.canvas.height.toString() + "px";
            document.getElementById("container").style.width  = this.ctx.canvas.width.toString()  + "px";

            //In how we draw our field image, WPIlib convention puts the origin in the bottom right of the image
            this.orig_px_x = this.ctx.canvas.width;
            this.orig_px_y = this.ctx.canvas.height;

            //Configure the appearance 
            this.ctx.fillStyle = FIELD_COLOR;
            //Draw polygon based on specified points 
            this.ctx.beginPath();
            for (i = 0; i < FIELDPOLY_FT.length; i++) {
                [x_px, y_px] = actLocToPx(FIELDPOLY_FT[i][0], FIELDPOLY_FT[i][1], this.orig_px_x, this.orig_px_y)

                if (i == 0) {
                    this.ctx.moveTo(x_px, y_px);
                } else {
                    this.ctx.lineTo(x_px, y_px);
                }
            }
            
            this.ctx.closePath();
            this.ctx.fill();

            this.ctx.drawImage(image,0,0,bg_image_width_px, bg_image_height_px);

            
            //Save robot dimensions
            ROBOT_W_PX = ROBOT_W_FT * PX_PER_FOOT;
            ROBOT_L_PX = ROBOT_L_FT * PX_PER_FOOT;

            //Fire up a new DAQ for the robot to capture the required signals.
            startNewDAQ("robotPose", sigIdList, 50);
            poseViewInitCompleted = true;
        }
    }
}

function poseDataHandler(data) {

    //Grab a reference to the canvases
    this.canvas = document.getElementById("field_bg_canvas");
    this.ctx = this.canvas.getContext("2d");

    this.canvas_robot = document.getElementById("robot_canvas");
    this.ctx_robot = this.canvas_robot.getContext("2d");

    this.canvas_path = document.getElementById("path_canvas");
    this.ctx_path = this.canvas_path.getContext("2d");

    if (data.daq_id == "robotPose") {

        sigValList = getSignalValuesFromIDs(sigIdList, data);

        desPoseX = sigValList[0];
        desPoseY = sigValList[1];
        desPoseT = sigValList[2];
        actPoseX = sigValList[3];
        actPoseY = sigValList[4];
        actPoseT = sigValList[5];
        estPoseX = sigValList[6];
        estPoseY = sigValList[7];
        estPoseT = sigValList[8];

        this.ctx_robot.clearRect(0, 0, this.canvas_robot.width, this.canvas_robot.height);

        if (estPoseX != null &&
            estPoseY != null &&
            estPoseT != null) {
            //Handle robot Estimated pose update
            [poseX_px, poseY_px]  = actLocToPx(estPoseX,estPoseY, this.orig_px_x, this.orig_px_y);
            drawRobot(this.ctx_robot, poseX_px, poseY_px, estPoseT, DRAW_STYLE_ESTIMATED);
            //draw new line segment
            drawPathSegment(this.ctx_path, poseX_px, poseY_px,botPrevEstPoseX,botPrevEstPoseY,DRAW_STYLE_ESTIMATED);
            botPrevEstPoseX = poseX_px;
            botPrevEstPoseY = poseY_px;
        }

        if (actPoseX != null &&
            actPoseY != null &&
            actPoseT != null) {
            //Handle robot Actual pose update
            [poseX_px, poseY_px]  = actLocToPx(actPoseX, actPoseY, this.orig_px_x, this.orig_px_y);
            drawRobot(this.ctx_robot, poseX_px, poseY_px, actPoseT, DRAW_STYLE_ACTUAL);
            //Draw new line segment
            drawPathSegment(this.ctx_path, poseX_px, poseY_px,botPrevActPoseX,botPrevActPoseY,DRAW_STYLE_ACTUAL);
            botPrevActPoseX = poseX_px;
            botPrevActPoseY = poseY_px;
        }

        if (desPoseX != null &&
            desPoseY != null &&
            desPoseT != null) {
            //Handle robot Desired pose update
            [poseX_px, poseY_px]  = actLocToPx(desPoseX,desPoseY, this.orig_px_x, this.orig_px_y);
            drawRobot(this.ctx_robot, poseX_px, poseY_px, desPoseT, DRAW_STYLE_DESIRED);
            //draw new line segment
            drawPathSegment(this.ctx_path, poseX_px, poseY_px,botPrevDesPoseX,botPrevDesPoseY,DRAW_STYLE_DESIRED);
            botPrevDesPoseX = poseX_px;
            botPrevDesPoseY = poseY_px;
        }

    }
}

drawRobot = function (ctx_in, x_pos_px, y_pos_px, rotation_deg, drawStyle) {

    //Draw the robot itself

    //Tweak rotation to match the javascript canvas draw angle convention
    rotation_deg *= -1;

    //Rotate to robot reference frame
    ctx_in.translate(x_pos_px, y_pos_px);
    ctx_in.rotate(rotation_deg * Math.PI / 180);

    //Draw robot body
	if(drawStyle == DRAW_STYLE_ACTUAL){
        //Solid filled in red robot is for Actual
        ctx_in.beginPath();
        ctx_in.strokeStyle = "black";
        ctx_in.lineWidth = "1";
        ctx_in.rect(-ROBOT_W_PX / 2, -ROBOT_L_PX / 2, ROBOT_W_PX, ROBOT_L_PX);
        ctx_in.closePath();
        ctx_in.stroke();
        ctx_in.fillStyle = "red";
        ctx_in.fillRect(-ROBOT_W_PX / 2, -ROBOT_L_PX / 2, ROBOT_W_PX, ROBOT_L_PX);
        
	} else if (drawStyle == DRAW_STYLE_ESTIMATED){
        ctx_in.beginPath();
        ctx_in.strokeStyle = "green";
        ctx_in.lineWidth = "3";
        ctx_in.rect(-ROBOT_W_PX / 2, -ROBOT_L_PX / 2, ROBOT_W_PX, ROBOT_L_PX);
        ctx_in.closePath();
        ctx_in.stroke();

        ctx_in.beginPath();
        ctx_in.strokeStyle = "white";
        ctx_in.lineWidth = "1";
        ctx_in.rect(-ROBOT_W_PX / 2, -ROBOT_L_PX / 2, ROBOT_W_PX, ROBOT_L_PX);
        ctx_in.closePath();
        ctx_in.stroke();
    } else {
        //Outlined blue robot is for Desired
        ctx_in.beginPath();
        ctx_in.strokeStyle = "cyan";
        ctx_in.lineWidth = "3";
        ctx_in.rect(-ROBOT_W_PX / 2, -ROBOT_L_PX / 2, ROBOT_W_PX, ROBOT_L_PX);
        ctx_in.closePath();
        ctx_in.stroke();

        ctx_in.beginPath();
        ctx_in.strokeStyle = "black";
        ctx_in.lineWidth = "1";
        ctx_in.rect(-ROBOT_W_PX / 2, -ROBOT_L_PX / 2, ROBOT_W_PX, ROBOT_L_PX);
        ctx_in.closePath();
        ctx_in.stroke();
    }
    
    //Draw front-of-robot arrowhead
    drawArrowhead(ctx_in, 0, ROBOT_L_PX / 2, 0, -ROBOT_L_PX / 3, 8);
    
    //Undo rotation
    ctx_in.rotate(-1 * rotation_deg * Math.PI / 180);
    ctx_in.translate(-x_pos_px, -y_pos_px);

    //Draw robot centroid marker
    ctx_in.beginPath();
    ctx_in.strokeStyle = "black";
    ctx_in.lineWidth = "1";
    ctx_in.moveTo(x_pos_px-5, y_pos_px);
    ctx_in.lineTo(x_pos_px+5, y_pos_px);
    ctx_in.moveTo(x_pos_px, y_pos_px-5);
    ctx_in.lineTo(x_pos_px, y_pos_px+5);
    ctx_in.closePath();
    ctx_in.stroke(); 
}

drawPathSegment = function(ctx_in, x_start_px, y_start_px, x_end_px, y_end_px, drawStyle){
    
    if(x_end_px >= 0 && y_end_px >= 0){
        //Draw the line segment for the most recent path taken
        ctx_in.beginPath();
        if(drawStyle == DRAW_STYLE_ACTUAL){
            ctx_in.strokeStyle = "red";
        } else if(drawStyle == DRAW_STYLE_DESIRED){
            ctx_in.strokeStyle = "cyan";
        } else { //Estimated
            ctx_in.strokeStyle = "green";
        }
        ctx_in.lineWidth = "1";
        ctx_in.moveTo(x_start_px, y_start_px);
        ctx_in.lineTo(x_end_px, y_end_px);
        ctx_in.closePath();
        ctx_in.stroke(); 
    }
}

function handlePathClearBtnClick() {
    this.ctx_path.clearRect(0, 0, this.canvas_path.width, this.canvas_path.height);
}


function drawArrowhead(context_in, from_x, from_y, to_x, to_y, radius) {
    var x_center = to_x;
    var y_center = to_y;

    var angle;
    var x;
    var y;

    context_in.beginPath();
    context_in.strokeStyle = "white";
    context_in.lineWidth = "-4";
    context_in.fillStyle = '#000';

    angle = Math.atan2(to_y - from_y, to_x - from_x)
    x = radius * Math.cos(angle) + x_center;
    y = radius * Math.sin(angle) + y_center;

    context_in.moveTo(x, y);

    angle += (1.0 / 3.0) * (2 * Math.PI)
    x = radius * Math.cos(angle) + x_center;
    y = radius * Math.sin(angle) + y_center;

    context_in.lineTo(x, y);

    angle += (1.0 / 3.0) * (2 * Math.PI)
    x = radius * Math.cos(angle) + x_center;
    y = radius * Math.sin(angle) + y_center;

    context_in.lineTo(x, y);

    context_in.closePath();

    context_in.fill();
}

registerOpenCallback(requestSignalList);
registerDAQUpdateCallback(poseDataHandler);
registerSignalListCallback(poseSignalListHandler);