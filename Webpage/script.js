var webSocketIP = "127.0.0.1"; //The IP of the websocket server.
var webSocketPort = "12345"; //The port the websocket is being served on.
var webSocketPath = "/"; //The file path of the websocket.
var formattedDataStringStandardArrayLength = 46; //The expected length of the formatted data string from formatRawMessage().
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

var canvas, context, dataArea, updateZoomButton, enterZoomTextArea, enterZoomButton, autoZoomButton, startButton; //These are global variables used for UI stuff.

function setup() { //Call this to get the program going.
	canvas = document.getElementById("mainCanvas"); //Grab the HTMl of the canvas.
	canvas.style.transform = "matrix(0, -1, 1, 0, 0, 0)"; //Rotate the canvas so up is forward, like in a map.
	context = canvas.getContext("2d"); //All canvas drawings are done through a context.
	context.fillStyle = "white"; //Set the fill style of closed shapes on the canvas to white.
	context.beginPath(); //This starts a path so lines can be drawn.

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
	formatted = formatRawMessage(data); //This takes the raw data sent through the websocket, and converts it into something that's a bit easier to use.
	console.log(formatted);

	if(formatted.length == formattedDataStringStandardArrayLength) { //The formatted data should be an array with 46 units. If the array is a different length, something is wrong.
		
		//Position Data**************************************************

		//Store the x, y, and z position in a separate variable.
		var positionXYZ = [formatted[formattedDataStringPositionIndeces[0]], formatted[formattedDataStringPositionIndeces[1]], formatted[formattedDataStringPositionIndeces[2]]];

		//Store the x, y, z, and w quaternion in a separate variable.
		var quaternionXYZW = [formatted[formattedDataStringQuaternionIndeces[1]], formatted[formattedDataStringQuaternionIndeces[0]], formatted[formattedDataStringQuaternionIndeces[2]], formatted[formattedDataStringQuaternionIndeces[3]]];

		//Unfortunately, there's still more formatting to be done.
		//The data entries stored in positionXYZ and quaternionXYZW are stored as strings, and have a bit before the number (e.g. "x:####").
		//The "x:" part is removed by slicing from the character in position 2, and then the Number() function is called to convert the string to a useable number.

		for(var i=0; i<positionXYZ.length; ++i) { //Format positionXYZ.
			positionXYZ[i] = Number(positionXYZ[i].slice(2));
		}
		for(var i=0; i<quaternionXYZW.length; ++i) { //Format quaternionXYZW.
			quaternionXYZW[i] = Number(quaternionXYZW[i].slice(2));
		}

		var eulerAngles = quaternionToEuler(quaternionXYZW); //Convert the quaternion to euler angles.
		var theta = eulerAngles[eulerAngleUsed]; //This is the XY-plane angle actually used, rotated 90 degrees so that forward is up instead of right.

		pointsRecord.push([positionXYZ[0], positionXYZ[1]]); //Store the next point to the list.

		//Scan Data**************************************************

		//Store the minimum and maximum angle ranges, and the scan data.
		var angleMin = formatted[37];
		var angleMax = formatted[38];
		var rangeList = formatted[44];
		var angleIncrement; //Calculate this later.

		//Get rid of the text in front of angles.
		angleMin = Number(angleMin.slice(10));
		angleMax = Number(angleMax.slice(10));
		
		//Convert the string of an array of numbers into an array of numbers.
		rangeList = rangeList.split(","); //Split along commas.
		rangeList[0] = rangeList[0].slice(8); //Remove the characters at the beginning.
		rangeList[rangeList.length-1] = rangeList[rangeList.length-1].slice(0, -1); //Get rid of that one last character.
		for(var i=0; i<rangeList.length; ++i) {
			rangeList[i] = Number(rangeList[i]); //Make them all numbers!
		}

		angleIncrement = (angleMax - angleMin) / rangeList.length;

		//console.log(angleMin);
		//console.log(angleMax);
		//console.log(rangeList);

		var xyRobotRangeList = [];

		for(var i=0; i<rangeList.length; ++i) {
			var scanTheta = angleMin + (i * angleIncrement); //Calculate the angle that the current range value is using.

			//Convert it to x, y form relative to the robot.
			xyRobotRangeList[i] = [0, 0];
			xyRobotRangeList[i][0] = rangeList[i] * Math.cos(scanTheta); //x=rcos(θ)
			xyRobotRangeList[i][1] = rangeList[i] * -Math.sin(scanTheta); //y=rsin(θ)
		}

		scanRecord.push(xyRobotRangeList);

		//Display**************************************************

		if(autoZoom) {
			updateZoom(); //If autozoom is selected, automatically set the zoom.
		}

		context.lineWidth = 1/scaleFactor; //Make sure the lines don't freak out.

		clearCanvas();
		drawRobotMarker();
		drawCurrentMap(positionXYZ, xyRobotRangeList);
		drawRobotPath(positionXYZ, theta);

		window.setTimeout(sendDataRequest, 100); //When using recorded data, use window.setTimeout().
		//requestAnimationFrame(sendDataRequest); //When using data directly from the robot, use requestAnimationFrame().
		//If you use requestAnimationFrame(), comment out EVERY debug statement, or the page will hang pretty quickly.
	}
	else { //Ok, so there's a problem with the data...
		console.log("Improper data received!"); //Tell me wtf is going on.
		console.log(data); //Print if out.
		requestAnimationFrame(sendDataRequest); //Try again, to see if it gets better.
	}
}

function sendDataRequest() {
	ws.send("ready");
	//When this message is sent, the server knows that the webpage is ready to process more data.
	//The server will then proceed to send the most recent data avaiable.
}
function formatRawMessage(raw) { //This takes the raw message and formats it in a way that the data can be accessed more easily.
	var refined = raw.split("\n"); //The data sent by the websocket has lots of newline characters -- every single piece of data is on its own line. This splits it apart into an array.
	for(var i=0; i<refined.length; ++i) { //Now, for each item that was split apart...
		refined[i] = refined[i].replace(/\s/g, ""); //Remove all the whitespace by replacing spaces with (quite literally) nothing.
	}
	return refined; //Spit it back at me.
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
function clearCanvas() {
	context.setTransform(1, 0, 0, 1, 0, 0); //Reset all transforms on the context.
	context.clearRect(0, 0, canvas.width, canvas.height); //Clear the canvas.
	context.transform(1, 0, 0, 1, canvas.width/2, canvas.height/2); //Put 0, 0 in the center of the canvas.
	context.transform(scaleFactor, 0, 0, scaleFactor, 0, 0); //Scale the canvas.
	context.transform(1, 0, 0, -1, 0, 0); //Flip the canvas so y+ is up.
}
function drawRobotPath(currentPosition, angle) {
	context.transform(Math.cos(-angle), Math.sin(-angle), -Math.sin(-angle), Math.cos(-angle), 0, 0); //Orient the path behind the robot properly.
	context.moveTo(pointsRecord[0][0]-currentPosition[0], pointsRecord[0][1]-currentPosition[1]); //Move to the first point in the path.
	context.beginPath();
	for(var i=1; i<pointsRecord.length; ++i) { //This draws lines from point i to point i-1
		context.lineTo(pointsRecord[i][0]-currentPosition[0], pointsRecord[i][1]-currentPosition[1]); //Draw a line to the next point.
		context.stroke();
	}
}
function drawCurrentMap(currentPosition, scan) {
	context.moveTo(scan[0][0], scan[0][1]);
	context.beginPath();
	for(var i=1; i<scan.length; ++i) {
		if(isNaN(scan[i][0]) || isNaN(scan[i][1]) || distance([scan[i][0], scan[i][1]], [scan[i-1][0], scan[i-1][1]]) > distanceDisplayThreshold) {
			context.moveTo(scan[i][0], scan[i][1]);
			context.beginPath();
		}
		else {
			context.lineTo(scan[i][0], scan[i][1]);
			context.stroke();
		}
	}
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