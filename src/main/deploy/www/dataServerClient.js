//Note - this PORT string must be aligned with the port the webserver is served on.
var port = "5806";
var hostname = window.location.hostname + ":" + port;

var dataSocket = null;

var onDAQUpdateCallbacks = [];
var onSignalListCallbacks = [];
var onOpenCallbacks = [];

var numTransmissions = 0;

//Automatically make the websockets connection on script inclusion
window.onload = function() {
    dataSocket = new WebSocket("ws://" + hostname + "/ds")
    numTransmissions = 0;

    dataSocket.onopen = function (event) {
        document.getElementById("id01").innerHTML = "Socket Open";
        for(var i = 0; i < onOpenCallbacks.length; i++){
            cbFcn = onOpenCallbacks[i];
            cbFcn();
        }
    };
    
    dataSocket.onmessage = function (event) {
        procData(event.data);
        numTransmissions = numTransmissions + 1;
        document.getElementById("id01").innerHTML = "COM Status: Socket Open. RX Count:" + numTransmissions;
    };
    
    dataSocket.onerror = function (error) {
        document.getElementById("id01").innerHTML = "COM Status: Error with socket. Reconnect to robot, open driver station, then refresh this page.";
        alert("ERROR from Robot PoseView: Robot Disconnected!!!\n\nAfter connecting to the robot, open the driver station, then refresh this page.");
    };
    
    dataSocket.onclose = function (error) {
        document.getElementById("id01").innerHTML = "COM Status: Error with socket. Reconnect to robot, open driver station, then refresh this page.";
        alert("ERROR from Robot PoseView: Robot Disconnected!!!\n\nAfter connecting to the robot, open the driver station, then refresh this page.");
    };
    
}

function requestSignalList(){
    // Send the command to get the list of all signals
    dataSocket.send(JSON.stringify({ cmd: "getSig" }));
}

function startNewDAQ(daq_id, signalIdList, rate_ms){
    //Fire up a new DAQ for the robot

    var daq_request_cmd = {};

    daq_request_cmd.cmd = "addDaq";
    daq_request_cmd.id = daq_id;
    daq_request_cmd.tx_period_ms = rate_ms.toString(); 
    daq_request_cmd.samp_period_ms = "0";
    daq_request_cmd.sig_id_list = signalIdList;

    //Request data from robot
    var sendVal = JSON.stringify(daq_request_cmd);
    dataSocket.send(sendVal);

    var sendVal = JSON.stringify({ cmd: "start", id: daq_id});
    dataSocket.send(sendVal);
}

function stopDAQ(daq_id){
    var sendVal = JSON.stringify({ cmd: "stop", id: daq_id});
    dataSocket.send(sendVal);
}

function procData(json_data) {
    data = JSON.parse(json_data);
    if (data.type == "sig_list") {
        for(var i = 0; i < onSignalListCallbacks.length; i++){
            cbFcn = onSignalListCallbacks[i];
            cbFcn(data);
        }
    } else if (data.type == "daq_update") {
        for(var i = 0; i < onDAQUpdateCallbacks.length; i++){
            cbFcn = onDAQUpdateCallbacks[i];
            cbFcn(data);
        }
    }
}

/**
 * Scripts should register their own functions against
 * different data pieces getting recieved.
 * @param {function} callback 
 */
function registerDAQUpdateCallback(callback){
    onDAQUpdateCallbacks.push(callback)
}

function registerSignalListCallback(callback){
    onSignalListCallbacks.push(callback)
}

function registerOpenCallback(callback){
    onOpenCallbacks.push(callback)
}

//Converts a list of signal Names into UUID's which can be requested in a DAQ.
// Needs "data" input that represents a signal list.
function getSignalIDsFromNames(signalNameList, data){
    var retArray = [];

    for (sigNameIdx = 0; sigNameIdx < signalNameList.length; sigNameIdx++){
        var found = false;
        for (i = 0; i < data.signals.length; i++) {
            if (data.signals[i].display_name == signalNameList[sigNameIdx]) {
                retArray.push(data.signals[i].id)
                found = true;
                break;
            } 
        }
        if(found == false){
            retArray.push(null);
        }
    }
    return retArray;
}

//Given a list of signal ID's, get a list of their values.
// Needs a data input that represents a DAQ update
function getSignalValuesFromIDs(signalIDList, data){
    var retArray = [];

    for (sigIdIdx = 0; sigIdIdx < signalIDList.length; sigIdIdx++){
        var found = false;
        for (i = 0; i < data.signals.length; i++) {
            var signal = data.signals[i];
            if (signal.samples.length > 0) {
                if (signal.id == signalIDList[sigIdIdx]) {
                    found = true;
                    retArray.push(signal.samples[signal.samples.length - 1].val);
                    break;
                }
            }
        }
        if(found == false){
            retArray.push(null);
        }
    }
    return retArray;
}