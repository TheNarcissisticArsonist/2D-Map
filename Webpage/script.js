var webSocketIP = "127.0.0.1"; //The IP of the websocket server.
var webSocketPort = "12345"; //The port the websocket is being served on.
var webSocketPath = "/"; //The file path of the websocket.
var formattedDataStringExpectedArrayLength = 46; //The expected length of the formatted data string from formatRawMessage().
var formattedDataStringPositionIndeces = [10, 11, 12]; //The location of the xyz position data in the formatted data string.
var formattedDataStringQuaternionIndeces = [14, 15, 16, 17]; //The location of the xyzw quaternion data in the formatted data string.
var eulerAngleUsed = 0; //Due to some weirdness with the robot's orientation data, the Euler angle for the XY-plane can be unexpected.
						//0 is roll, 1 is pitch, 2 is yaw.
var robotMarkerRadius = 0.3; //The radius of the circle that marks the robot's location, in meters.
var robotMarkerArrowAngle = Math.PI/6; //There's an arrow on the circle, showing which direction the robot is pointing. This is the angle between the centerline and one of the sides.
var scaleFactorMultiplier = 50; //This lets it default to 1 pixel = 2 cm.
var distanceDisplayThreshold = 0.1; //

var pointsRecord = []; //This is the list of 2D points where the robot has been, so the program can draw lines between them.
var scaleFactor = 50; //As the path and information get bigger, it's useful to zoom out.
var positionOffset = [0, 0]; //This is used to keep the robot's location on the screen centered.
var pathMaxLength = Infinity; //If the program ever starts to get slow, this can be used to begin erasing points from the beginning of the path.
						 	  //I'll set it to something once I find that point.
var autoZoom = false; //This can be toggled. If it's true, the map will scale automatically so everything is visible.
var scanRecord = []; //This is the list of laser scans. The indeces correspond with pointsRecord[]. //They're in x-y position format, the same as pointsRecord.
var angleMinIndex = 37; //This is the formatted array index of the minimum angle for the scan.
var angleMaxIndex = 38; //Same, but for the maximum angle.
var rangeListIndex = 44; //Same, but for the list of ranges.
var saveThisScan = false; //If true, the next scan will be saved.
var minimumPositionDistanceToRecord = 0.001 //If the distance between two position samples is less than this, only one of the points will be kept.

var canvas, context, dataArea, updateZoomButton, enterZoomTextArea, enterZoomButton, autoZoomButton, startButton; //These are global variables used for UI stuff.

function setup() { //Call this to get the program going
	canvas = document.getElementById("mainCanvas"); //Grab the HTML canvas element.
	canvas.style.transform = "matrix(0, -1, 1, 0, 0, 0)"; //Rotate the canvas so up is forward, like in a map.
	context = canvas.getContext("2d"); //All canvas drawings are done through a context.
	context.fillStyle = "white"; //Set the fill style of closed shapes on the canvas to white.
	context.beginPath(); //This starts a path with which lines can be drawn.

	dataArea = document.getElementById("dataPrintout"); //As the program receives data, this area on the webpage can be used to record it.

	ws = new WebSocket("ws://"+webSocketIP+":"+webSocketPort+webSocketPath); //This creates the websocket object.
	ws.onmessage = function(event) { //When a message is received...
		//console.log(event.data);
		mainLoop(event.data); //Go into the main loop and use the data.
	}
	ws.onopen = function() {
		console.log("Connection opened.");
		sendDataRequest(); //Send a request for data once the connection is opened.
	}
}
function mainLoop(data) {
	formattedMessage = formatRawMessage(data); //This takes the raw data sent through the websocket, and converts it into something that's a bit easier to use.
	//console.log(formatted);

	if(formattedMessage.length == formattedDataStringExpectedArrayLength) {
		var positionXYZ = getPositionFromFormattedMessage(formattedMessage);
		var quaternion = getQuaternionFromFormattedMessage(formattedMessage);
		var eulerAngles = quaternionToEuler(quaternion);
		var theta = eulerAngles[eulerAngleUsed];
		var position = [positionXYZ[0], positionXYZ[1]]

		if(pointsRecord.length == 0) {
			pointsRecord.push(position);
		}
		else if(distance(position, pointsRecord[pointsRecord.length - 1]) < minimumPositionDistanceToRecord) {
			pointsRecord[pointsRecord.length - 1] = position;
		}
		else {
			pointsRecord.push(position);
		}

		var angleMin = formattedMessage[angleMinIndex];
		var angleMax = formattedMessage[angleMaxIndex];
		var rangeList = formattedMessage[rangeListIndex];
		var angleIncrement; //This is calculated, not read.

		//Format all of the scan data.
		var scanDataFormatted = cleanUpScanData(angleMin, angleMax, rangeList);
		angleMin = scanDataFormatted[0];
		angleMax = scanDataFormatted[1];
		rangeList = scanDataFormatted[2];
		angleIncrement = (angleMax - angleMin) / rangeList.length;

		var xyRangeList = convertRangeListToXY(angleMin, angleMax, angleIncrement, rangeList, position, theta);

		if(saveThisScan || scanRecord.length == 0) {
			scanRecord.push(xyRangeList);
			saveThisScan = false;
		}
		else {
			scanRecord[scanRecord.length - 1] = xyRangeList;
		}

		if(autoZoom) {
			updateZoom(); //If autozoom is selected, automatically set the zoom.
		}

		context.lineWidth = 1/scaleFactor;

		clearCanvas();
		setConstantCanvasTransforms();
		drawRobotMarker();
		setCanvasTransforms(position, theta);
		drawRobotPath();
		drawRobotMap();
	}
	else {
		console.log("Error! The message is of an unexpected format!");
		console.log(formattedMessage);
	}

	requestAnimationFrame(sendDataRequest);
}

function formatRawMessage(raw) { //This takes the raw message and formats it in a way that the data can be accessed more easily.
	var refined = raw.split("\n"); //The data sent by the websocket has lots of newline characters -- every single piece of data is on its own line. This splits it apart into an array.
	for(var i=0; i<refined.length; ++i) { //Now, for each item that was split apart...
		refined[i] = refined[i].replace(/\s/g, ""); //Remove all the whitespace by replacing spaces with (quite literally) nothing.
	}
	return refined; //Spit it back at me.
}
function getPositionFromFormattedMessage(formattedMessage) {
	var position = [0, 0, 0]; //[x, y, z]

	//These are the locations of the three relavent data values.
	position[0] = formattedMessage[formattedDataStringPositionIndeces[0]];
	position[1] = formattedMessage[formattedDataStringPositionIndeces[1]];
	position[2] = formattedMessage[formattedDataStringPositionIndeces[2]];

	//This removes the stuff around the number itself in the string, and actually converts it to a number.
	for(var i=0; i<position.length; ++i) {
		position[i] = Number(position[i].slice(2));
	}

	return position;
}
function getQuaternionFromFormattedMessage(formattedMessage) {
	var quaternion = [0, 0, 0, 0]; //[x, y, z, w]

	//These are the locations of the four relavent data values.
	quaternion[0] = formattedMessage[formattedDataStringQuaternionIndeces[0]];
	quaternion[1] = formattedMessage[formattedDataStringQuaternionIndeces[1]];
	quaternion[2] = formattedMessage[formattedDataStringQuaternionIndeces[2]];
	quaternion[3] = formattedMessage[formattedDataStringQuaternionIndeces[3]];
	
	//This removes the stuff around the number itself in the string, and actually converts it to a number.
	for(var i=0; i<quaternion.length; ++i) {
		quaternion[i] = Number(quaternion[i].slice(2));
	}

	return quaternion;
}
function quaternionToEuler(quat) { //This takes the quaternion array [x, y, z, w] and returns the euler array [φ, θ, ψ]
	//The quaternion describes the orientation and rotation of the robot, but it's very complicated.
	//These formulas convert the XYZW quaternion into φθψ (phi-theta-psi) euler angles.
	//It's easiest to think of them as φ=pitch (index 0), θ=roll (index 1), and ψ=yaw (index 2).
	//These formulas were taken from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_from_Quaternion

	euler = [];
	euler[0] = Math.atan2(2*((quat[0]*quat[1]) + (quat[2]*quat[3])), 1-(2*((quat[1]*quat[1]) + (quat[2]*quat[2]))));
	euler[1] = Math.asin(2*((quat[0]*quat[2]) - (quat[3]*quat[1])));
	euler[2] = Math.atan2(2*((quat[0]*quat[3]) + (quat[1]*quat[2])), 1-(2*((quat[2]*quat[2]) + (quat[3]*quat[3]))));
	return euler;
}
function cleanUpScanData(min, max, ranges) {
	var angleMin = Number(min.slice(10));
	var angleMax = Number(max.slice(10));

	var rangeList = ranges.split(","); //Split along commas.
	rangeList[0] = rangeList[0].slice(8); //Remove the characters at the beginning.
	rangeList[rangeList.length - 1] = rangeList[rangeList.length - 1].slice(0, -1); //Get rid of that one last character.
	for(var i=0; i<rangeList.length; ++i) {
		rangeList[i] = Number(rangeList[i]); //Make them all numbers!
	}

	return [angleMin, angleMax, rangeList];
}
function convertRangeListToXY(min, max, increment, ranges, currentPosition, theta) {
	var scanList = [];
	var currentScan = [];
	var scanTheta, x, y, x1, y1;
	for(var i=0; i<ranges.length; ++i) {
		scanTheta = min + (i * increment);

		//Convert it from polar to cartesian.
		currentScan[0] = ranges[i] * Math.cos(scanTheta);
		currentScan[1] = ranges[i] * -Math.sin(scanTheta);

		//Rotate it so that it's relative to the origin.
		x = currentScan[0];
		y = currentScan[1];
		x1 = (x * Math.cos(-theta)) + (y * Math.sin(-theta));
		y1 = (-1 * x * Math.sin(-theta)) + (y * Math.cos(-theta));
		currentScan[0] = x1;
		currentScan[1] = y1;

		//Translate it so that it's relative to the origin.
		currentScan[0] += currentPosition[0];
		currentScan[1] += currentPosition[1];

		var dataToPush = [currentScan[0], currentScan[1]];

		//Push it into the scanList array.
		scanList.push(dataToPush);
	}

	return scanList;
}
function updateZoom() {
	//This function iterates through every point, determining which point is the furthest away from the current location.
	//Then, it sets the zoom so that the entire map is visible.
	var furthestPointIndex = 0;
	var furthestDistance = distance(pointsRecord[0], pointsRecord[pointsRecord.length-1]);
	var maxDistance = (canvas.width/2) - (canvas.width/100); //This makes sure there's a little buffer space around the robot's path.

	for(var i=1; i<pointsRecord.length-1; ++i) {
		currentDistance = distance(pointsRecord[i], pointsRecord[pointsRecord.length-1]); //This calculates the distance between the selected point and the most recent point.
		if(currentDistance >= furthestDistance) {
			furthestPointIndex = i;
			furthestDistance = currentDistance;
		}
	}
	scaleFactor = maxDistance/furthestDistance;
}
function distance(pointA, pointB) {
	//This is just an implementation of the distance formula.
	return Math.sqrt(Math.pow(pointB[0]-pointA[0], 2) + Math.pow(pointB[1]-pointA[1], 2));
}
function enterZoom() { //Change the scale factor based on user input.
	rawFactor = enterZoomTextArea.value;
	console.log(rawFactor);
	if(!isNaN(rawFactor)) { //Make sure it's a number.
		if(Number(rawFactor) > 0) { //Make sure it's positive.
			scaleFactor = scaleFactorMultiplier*Number(rawFactor);
		}
	}
}
function toggleAutoZoom() {
	//This is called when you click the button to toggle automatic zoom.
	autoZoom = !autoZoom;
}
function clearCanvas() {
	context.setTransform(1, 0, 0, 1, 0, 0); //Reset all transforms on the context.
	context.clearRect(0, 0, canvas.width, canvas.height); //Clear the canvas.
	context.beginPath();
}
function setConstantCanvasTransforms() {
	context.transform(1, 0, 0, 1, canvas.width/2, canvas.height/2); //Put 0, 0 in the center.
	context.transform(1, 0, 0, -1, 0, 0); //Flip the canvas so y+ is up.
	context.transform(scaleFactor, 0, 0, scaleFactor, 0, 0); //Scale the canvas.
}
function drawRobotMarker() {
	//This will draw a circle around the center for the robot marker.
	context.beginPath();
	context.arc(0, 0, robotMarkerRadius, 0, 2*Math.PI);
	context.stroke();

	//These lines draw a triangle inside the circle, to show the direction of the robot.
	context.beginPath();
	context.moveTo(robotMarkerRadius*Math.cos(0), robotMarkerRadius*Math.sin(0));
	context.lineTo(robotMarkerRadius*Math.cos(Math.PI-robotMarkerArrowAngle), robotMarkerRadius*Math.sin(Math.PI-robotMarkerArrowAngle));
	context.stroke();
	context.moveTo(robotMarkerRadius*Math.cos(0), robotMarkerRadius*Math.sin(0));
	context.lineTo(robotMarkerRadius*Math.cos(Math.PI-robotMarkerArrowAngle), -robotMarkerRadius*Math.sin(Math.PI-robotMarkerArrowAngle));
	context.stroke();
	context.moveTo(robotMarkerRadius*Math.cos(Math.PI-robotMarkerArrowAngle), robotMarkerRadius*Math.sin(Math.PI-robotMarkerArrowAngle));
	context.lineTo(robotMarkerRadius*Math.cos(Math.PI-robotMarkerArrowAngle), -robotMarkerRadius*Math.sin(Math.PI-robotMarkerArrowAngle));
	context.stroke();
}
function setCanvasTransforms(position, theta) {
	context.transform(Math.cos(-theta), Math.sin(-theta), -Math.sin(-theta), Math.cos(-theta), 0, 0); //Rotate the context so the path is properly oriented.
	context.transform(1, 0, 0, 1, -position[0], -position[1]);
}
function drawRobotPath() {
	context.moveTo(pointsRecord[0][0], pointsRecord[0][1]);
	for(var i=1; i<pointsRecord.length; ++i) {
		context.lineTo(pointsRecord[i][0], pointsRecord[i][1]);
		context.stroke();
	}
}
function drawRobotMap() {
	for(var i=0; i<scanRecord.length; ++i) {
		scan = scanRecord[i];
		context.moveTo(scan[0][0], scan[0][1]);
		context.beginPath();
		for(var j=1; j<scan.length; ++j) {
			if(isNaN(scan[j][0]) || isNaN(scan[j][1]) || distance([scan[j][0], scan[j][1]], [scan[j-1][0], scan[j-1][1]]) > distanceDisplayThreshold) {
				context.moveTo(scan[j][0], scan[j][1]);
				context.beginPath();
			}
			else {
				context.lineTo(scan[j][0], scan[j][1]);
				context.stroke();
			}
		}
	}
}
function sendDataRequest() {
	ws.send("ready");
	//When this message is sent, the server knows that the webpage is ready to process more data.
	//The server will then proceed to send the most recent data avaiable.
}
function saveScan() {
	saveThisScan = true;
	//This comment is just here so the function will collapse in sublime.
}

//This actually sets it up so if you click "setup", the program starts.
startButton = document.getElementById("start");
startButton.addEventListener("click", setup);

//These are some of the html elements used.
updateZoomButton = document.getElementById("updateZoom");
updateZoomButton.addEventListener("click", updateZoom);
enterZoomTextArea = document.getElementById("youSetZoom");
enterZoomButton = document.getElementById("enterZoom");
enterZoomButton.addEventListener("click", enterZoom);
autoZoomButton = document.getElementById("autoZoom");
autoZoomButton.addEventListener("click", toggleAutoZoom);
document.getElementById("saveScan").addEventListener("click", saveScan);