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
var distanceDisplayThreshold = 0.1; //If the distance between two points in a scan is greater than 0.1, it won't draw a line between them.
var pointsRecord = []; //This is the list of 2D points where the robot has been, so the program can draw lines between them.
var scaleFactor = 50; //As the path and information get bigger, it's useful to zoom out.
var scanRecord = []; //This is the list of laser scans. The indeces correspond with pointsRecord[]. //They're in x-y position format, the same as pointsRecord.
var scanThetaMinIndex = 37; //This is the formatted array index of the minimum angle for the scan.
var scanThetaMaxIndex = 38; //Same, but for the maximum angle.
var scanRangeListIndex = 44; //Same, but for the list of ranges.
var saveThisScan = false; //If true, the next scan will be saved.
var angleOffset = 0; //Calculated with ICP to correct the robot's orientation.
var positionOffset = [0, 0]; //Ditto the above, but for position.
var minimumPositionDistanceToRecord = 0.01; //If the distance between two position samples is less than this, only one of the points will be kept. This is in meters.
var icpAverageDistanceTraveledThreshold = 0.01; //The average distance traveled per point must be less than this for ICP to finish.
var icpNoMovementCounterThreshold = 3; //ICP must lead to no movement at least this many times for it to finish.
var numberOfScansToCompare = 5; //How many scans are used for comparison when using ICP.
var scanDensityDistance = 0.01; //In meters, the minimum significant distance between two points.
var maxICPLoopCount = 250; //The maximum number of times ICP can run.
var minICPComparePoints = 1000; //The minimum number of points ICP must use to compare.
var maximumPointMatchDistance = 2; //The maximum distance between matched points for ICP.
var goodCorrespondenceThreshold = 0.01; //If during point matching, the distance between two matched points is less than this, don't test any further points for a closer match.
var currentlyScanning = true; //Used to know when to stop asking for more scans.
var lastAngle = 0; //Saved each iteration in case the user turns off scans. Updated in mainLoop.
var lastPosition = [0, 0]; //Saved each iteration, for the same reason as above. Updated in drawRobotPath.
var numFailedScans = 0; //How many scans have failed in a row.
var maxNumFailedScans = 10; //How many scans have to fail in a row to go to user-manual fit.
var currentMouseCoords = [0, 0]; //The current coordinates of the mouse.
var canvasClickedCoords = [0, 0]; //The coordinates where the canvas was clicked.
var wasTheCanvasClicked = false; //Pretty self explanatory.
var overallCanvasDrag = [0, 0]; //This is applied to context transforms, so you can drag the map.
var lastOverallCanvasDrag = [0, 0]; //This is used so that when you drag the map, it's then applied to the next time you drag it.

var canvas, context, dataArea, updateZoomButton, enterZoomTextArea, enterZoomButton, autoZoomButton, startButton; //These are global variables used for UI stuff.
function setup() {
	console.log("Running setup function.");

	canvas = document.getElementById("mainCanvas"); //Grab the HTML canvas element.
	canvas.style.transform = "matrix(0, -1, 1, 0, 0, 0)"; //Rotate the canvas so up is forward, like in a map.
	context = canvas.getContext("2d"); //All canvas drawings are done through a context.
	context.fillStyle = "white"; //Set the fill style of closed shapes on the canvas to white.
	context.beginPath(); //This starts a path with which lines can be drawn.

	canvas.addEventListener("mousedown", canvasClicked);
	canvas.addEventListener("mouseup", canvasReleased);
	canvas.addEventListener("mouseleave", canvasReleased);
	document.body.addEventListener("mousemove", function(event) { mouseMoved(event); });

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
	var formattedMessage = formatRawMessage(data); //This takes the raw data sent through the websocket, and converts it into something that's a bit easier to use.

	if(!currentlyScanning) {
		notCurrentlyScanning(data);
		return;
	}
	else if(formattedMessage.length == formattedDataStringExpectedArrayLength) { //I.e., the data is correct...
		normalMainLoop(formattedMessage);
	}
	else {
		badDataMainLoop(formattedMessage);
	}

	requestAnimationFrame(sendDataRequest);
	//This sends a data request again, so the server will send the next piece of data, thereby calling the main loop again.
	//requestAnimationFrame is used so the browser has time to catch up.
}

function notCurrentlyScanning(data) {
	//This section of code can be activated by clicking the button to stop scanning on the webpage.
	//It allows you to zoom in and out, without taking new scans. Since ICP isn't being run, it renders much more quickly.
	console.log("Not currently scanning.");

	//These are saved, to keep the screen centered on where the robot would be. I may rework this later.
	var robotPosition = lastPosition;
	var robotOrientationTheta = lastAngle;

	context.lineWidth = 1 / scaleFactor;
	clearCanvas();
	setConstantCanvasTransforms();
	setCanvasTransforms(robotPosition, robotOrientationTheta);
	context.translate(overallCanvasDrag[1], -overallCanvasDrag[0]);
	drawRobotPath();
	drawRobotMap();
	requestAnimationFrame(function() { mainLoop(""); });
}
function normalMainLoop(formatted) {
	//console.log("Main loop!");

	var robotPositionXYZ = getPositionFromFormattedMessage(formatted);
	var quaternion = getQuaternionFromFormattedMessage(formatted);
	var eulerAngles = quaternionToEuler(quaternion); //The robot sends orientation data in the form of a quaternion, whereas euler angles (the normal kind) are what canvas uses.

	//This offsets the position and orientation by the stored error.
	var robotOrientationTheta = eulerAngles[eulerAngleUsed] - angleOffset;
	var robotPosition = numeric.sub(robotPositionXYZ.slice(0, 2), positionOffset);

	lastAngle = robotOrientationTheta; //This is used if the user stops scanning.
	
	var scanDataFormatted = cleanUpScanData(formatted[scanThetaMinIndex], formatted[scanThetaMaxIndex], formatted[scanRangeListIndex]); //This converts the strings into useable numbers and arrays.
	var scanThetaMin = scanDataFormatted[0];
	var scanThetaMax = scanDataFormatted[1];
	var scanRangeList = scanDataFormatted[2];
	var scanAngleIncrement = (scanThetaMax - scanThetaMin) / scanRangeList.length; //This information is actually in the message, but I prefer to calculate it myself.

	storePosition(robotPosition); //This appends the robotPosition to the pointsRecord array, and does absolutely nothing else (yet).
	processScanData(scanThetaMin, scanThetaMax, scanRangeList, scanAngleIncrement, robotPosition, robotOrientationTheta); //This is where the bulk of my computing time is, as this includes the ICP loop.

	context.lineWidth = 1 / scaleFactor; //This makes sure the lines don't get really thick when the context is zoomed in.

	clearCanvas();
	setConstantCanvasTransforms();
	drawRobotMarker();
	setCanvasTransforms(robotPosition, robotOrientationTheta);
	drawRobotPath();
	drawRobotMap();
}
function badDataMainLoop(formatted) {
	if(formatted[0] == "OLD" || formatted[formatted.length - 1] == "OLD") {
		//console.log("Error! The message is of an unexpected format! (Old data)");
	}
	else {
		console.log("Error! The message is of an unexpected format!\n" + formatted); //If it's not a good message, it's not an empty message, and it's not an old message, print whatever weird crap came through the socket.k
	}
}

function formatRawMessage(raw) { //This takes the raw message and formats it in a way that the data can be accessed more easily.
	var refined = raw.split("\n"); //The data sent by the websocket has lots of newline characters -- every single piece of data is on its own line. This splits it apart into an array.
	for(var i=0; i<refined.length; ++i) { //Now, for each item that was split apart...
		refined[i] = refined[i].replace(/\s/g, ""); //Remove all the whitespace by replacing spaces with (quite literally) nothing.
	}
	return refined; //Send me the refined message.
}
function sendDataRequest() {
	ws.send("ready");
	//When this message is sent, the server knows that the webpage is ready to process more data.
	//The server will then proceed to send the most recent data avaiable. If no new data is available, it will send a message to that effect.
}
function getPositionFromFormattedMessage(formattedMessage) {
	var position = [0, 0, 0]; //[x, y, z]

	//These are the locations of the three relavent data values. You can change these with the variables at the top.
	position[0] = formattedMessage[formattedDataStringPositionIndeces[0]];
	position[1] = formattedMessage[formattedDataStringPositionIndeces[1]];
	position[2] = formattedMessage[formattedDataStringPositionIndeces[2]];

	//This removes the stuff around the number itself in the string, and actually converts it to a number.
	//The actual raw value is something like "x:######".
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
	//The actual raw value is something like "x:######".
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

	var x = quat[0];
	var y = quat[1];
	var z = quat[2];
	var w = quat[3];

	euler = [];
	euler[0] = Math.atan2(2*((x*y) + (z*w)), 1-(2*((y*y) + (z*z))));
	euler[1] = Math.asin(2*((x*z) - (w*y)));
	euler[2] = Math.atan2(2*((x*w) + (y*z)), 1-(2*((z*z) + (w*w))));
	return euler;
}
function cleanUpScanData(min, max, rangeList) {
	//Trimming away some text before the number.
	min = Number(min.slice(10));
	max = Number(max.slice(10));

	rangeList = rangeList.split(","); //Split along commas.
	rangeList[0] = rangeList[0].slice(8); //Remove the characters at the beginning.
	rangeList[rangeList.length - 1] = rangeList[rangeList.length - 1].slice(0, -1); //Get rid of that one last character.
	for(var i=0; i<rangeList.length; ++i) {
		rangeList[i] = Number(rangeList[i]); //Make them all numbers!
	}

	return [min, max, rangeList];
}
function storePosition(position) {
	pointsRecord.push(position);
	//This is its own function because I might later want to downsample, or do some other manipulation on the point.
}
function processScanData(angleMin, angleMax, rangeList, angleIncrement, robotPosition, robotAngle) {
	//Throughout this function, the scan is gradually refined from a list of distances to a set of newfound points to add to the map.
	var robotXYList = convertScanToRobotXY(angleMin, angleMax, rangeList, angleIncrement);
	var globalXYList = convertRobotXYToGlobalXY(robotXYList, robotPosition, robotAngle);
	var cleanGlobalXYList = removeScanNaN(globalXYList);

	if(scanRecord.length > 0) {
		//This is only run when there's at least one scan already stored, or there would be nothing for ICP to compare to!
		//if(numFailedScans < maxNumFailedScans) {
			cleanGlobalXYList = runICP(cleanGlobalXYList);
		//}
		//else {
		//	manualFit(cleanGlobalXYList);
		//}
	}

	reducedGlobalXYList = removeDuplicates(cleanGlobalXYList);

	if(reducedGlobalXYList.length > 0) {
		scanRecord.push(cleanGlobalXYList);
	}
}
function convertScanToRobotXY(min, max, rangeList, increment) {
	var currentTheta, x, y;
	var scan = [];
	for(var i=0; i<rangeList.length; ++i) {
		currentTheta = min + (i * increment);
		x = rangeList[i] * Math.cos(currentTheta);
		y = rangeList[i] * Math.sin(currentTheta) * -1;
		scan.push([x, y]);
	}
	return scan;
}
function convertRobotXYToGlobalXY(scan, position, theta) {
	var x, y, x1, y1;
	for(var i=0; i<scan.length; ++i) {
		x = scan[i][0];
		y = scan[i][1];
		x1 = (x * Math.cos(-theta)) + (y * Math.sin(-theta));
		y1 = (-1 * x * Math.sin(-theta)) + (y * Math.cos(-theta));
		scan[i][0] = x1 + position[0];
		scan[i][1] = y1 + position[1];
	}
	return scan;
}
function removeScanNaN(scanList) {
	var cleanScan = [];
	for(var i=0; i<scanList.length; ++i) {
		if(!(isNaN(scanList[i][0]) || isNaN(scanList[i][1]))) {
			cleanScan.push(scanList[i]);
		}
	}
	return cleanScan;
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
function clearCanvas() {
	context.setTransform(1, 0, 0, 1, 0, 0); //Reset all transforms on the context.
	context.clearRect(0, 0, canvas.width, canvas.height); //Clear the canvas.
	context.beginPath();
}
function saveScan() {
	saveThisScan = true;
	//This comment is just here so the function will collapse in sublime.
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
function setCanvasTransforms(position, orientation) {
	context.transform(Math.cos(-orientation), Math.sin(-orientation), -Math.sin(-orientation), Math.cos(-orientation), 0, 0); //Rotate the context so the path is properly oriented.
	context.transform(1, 0, 0, 1, -position[0], -position[1]); //This is the translation to center the robot.
}
function drawRobotPath() {
	context.strokeStyle = "#00aa00";
	context.beginPath();
	//This will draw a line from each point in the robot's path to the subsequent point.
	context.moveTo(pointsRecord[0][0], pointsRecord[0][1]);
	var d;
	for(var i=1; i<pointsRecord.length; ++i) {
		d = distance([pointsRecord[i][0], pointsRecord[i][1]], [pointsRecord[i-1][0], pointsRecord[i-1][1]]);
		if(distance > 0.05) {
			context.moveTo(pointsRecord[i][0], pointsRecord[i][1]);
			context.beginPath();
		}
		else {
			context.lineTo(pointsRecord[i][0], pointsRecord[i][1]);
			context.stroke();
			lastPosition = [pointsRecord[i][0], pointsRecord[i][1]];
		}
	}
	context.strokeStyle = "#000000";
	context.beginPath();
}
function drawRobotMap() {
	context.strokeStyle = "#aa0000";
	context.beginPath();
	for(var i=0; i<scanRecord.length; ++i) {
		scan = scanRecord[i];
		context.moveTo(scan[0][0], scan[0][1]);
		context.beginPath();
		for(var j=1; j<scan.length; ++j) {
			if(distance([scan[j][0], scan[j][1]], [scan[j-1][0], scan[j-1][1]]) > distanceDisplayThreshold) {
				context.moveTo(scan[j][0], scan[j][1]);
				context.beginPath();
			}
			else {
				context.lineTo(scan[j][0], scan[j][1]);
				context.stroke();
			}
		}
	}
	context.strokeStyle = "#000000";
	context.beginPath();
}
function runICP(scan) {
	var currentScan = [];
	var knownPoints = [];
	var finished = false;
	var iterationTotalDistance = 0;
	var iterationAverageDistance = 0;
	var rotationMatrix, translation, icpFunctionOutput;
	var angle = 0;
	var pointPairs = [];
	var icpLoopCounter = 0;
	var pointSet1 = [];
	var pointSet2 = [];
	var oldScanPoints = [];
	var totalLoopCount = 0;
	var scanAngleError = 0;
	var scanPositionError = [0, 0];

	for(var i=0; i<scan.length; ++i) {
		currentScan.push(scan[i]);
	}
	var i = scanRecord.length - 1;
	while(i >= 0 && knownPoints.length <= minICPComparePoints) {
		knownPoints = knownPoints.concat(scanRecord[i]);
		--i;
	}
	while(!finished) {
		++totalLoopCount;
		if(totalLoopCount >= maxICPLoopCount) {
			console.log("Fail!");
			++numFailedScans;
			currentScan = [];
			scanAngleError = 0;
			scanPositionError = [0, 0];
			break;
		}
		iterationTotalDistance = 0;
		pointSet1 = [];
		pointSet2 = [];

		pointPairs = matchPoints(knownPoints, currentScan);
		for(var i=0; i<pointPairs.length; ++i) {
			pointSet1[i] = pointPairs[i][0];
			pointSet2[i] = pointPairs[i][1];
		}
		icpFunctionOutput = ICP(pointSet1, pointSet2);
		rotationMatrix = icpFunctionOutput.R;
		translation = icpFunctionOutput.T;
		angle = icpFunctionOutput.theta;

		for(var i=0; i<currentScan.length; ++i) {
			oldScanPoints[i] = currentScan[i];
			currentScan[i] = numeric.add(numeric.dot(rotationMatrix, currentScan[i]), translation);
			iterationTotalDistance += distance(oldScanPoints[i], currentScan[i]);
		}
		iterationAverageDistance = iterationTotalDistance / currentScan.length;

		if(iterationAverageDistance < icpAverageDistanceTraveledThreshold) {
			++icpLoopCounter;
			if(icpLoopCounter >= icpNoMovementCounterThreshold) {
				finished = true;
				console.log("Success!");
				numFailedScans = 0;
			}
			console.log("Good scan! " + iterationAverageDistance);
		}
		else {
			icpLoopCounter = 0;
			console.log("           " + iterationAverageDistance);
		}
		
		scanAngleError += angle;
		scanPositionError = numeric.add(scanPositionError, translation);
	}

	//console.log("Angle error: " + scanAngleError);
	//console.log("Position error: " + scanPositionError[0] + ", " + scanPositionError[1]);

	angleOffset -= scanAngleError;
	positionOffset = numeric.sub(positionOffset, scanPositionError);

	//console.log("Finished after " + totalLoopCount + "\n\n\n\n\n");

	return currentScan;
}
function ICP(set1, set2) {
	var pi = [];
	var pi1 = [];

	for(var i=0; i<set2.length; ++i) {
		pi.push(set1[i]);
		pi1.push(set2[i]);
	}

	var p = [0, 0];
	var p1 = [0, 0];

	for(var i=0; i<pi.length; ++i) {
		p[0] += pi[i][0];
		p[1] += pi[i][1];
		p1[0] += pi1[i][0];
		p1[1] += pi1[i][1];
	}
	p[0] = p[0]/pi.length;
	p[1] = p[1]/pi.length;
	p1[0] = p1[0]/pi1.length;
	p1[1] = p1[1]/pi1.length;

	var qi = [];
	var qi1 = [];

	for(var i=0; i<pi.length; ++i) {
		qi.push([pi[i][0]-p[0], pi[i][1]-p[1]]);
		qi1.push([pi1[i][0]-p1[0], pi1[i][1]-p1[1]]);
	}

	var H = [[0, 0], [0, 0]];
	for(var i=0; i<qi.length; ++i) {
		//H = Sum qi * qi1 (transpose)
		H[0][0] += qi1[i][0]*qi[i][0];
		H[0][1] += qi1[i][1]*qi[i][0];
		H[1][0] += qi1[i][0]*qi[i][1];
		H[1][1] += qi1[i][1]*qi[i][1];
	}

	var U;
	var A;
	var V;
	var Vt;
	var Ut;

	svdOutput = numeric.svd(H);
	U = svdOutput.U;
	A = svdOutput.S;
	V = svdOutput.V;
	Vt = numeric.transpose(V);
	Ut = numeric.transpose(U);

	rotationMatrix = numeric.dot(V, Ut);
	
	translation = [];
	Rp = numeric.dot(rotationMatrix, p);
	translation = numeric.sub(p1, Rp);

	//console.log(rotationMatrix);
	//console.log(translation);

	for(var i=0; i<2; ++i) {
		for(var j=0; j<2; ++j) {
			if(rotationMatrix[i][j] > 1) {
				rotationMatrix[i][j] = 1;
			}
			else if(rotationMatrix[i][j] < -1) {
				rotationMatrix[i][j] = -1;
			}
		}
	}

	return {R: rotationMatrix, T: translation, theta: Math.atan2(rotationMatrix[1][0], rotationMatrix[0][0])};
}
function matchPoints(set1, set2) {
	var indexPairs = [];
	
	for(var i=0; i<set2.length; ++i) {
		if(Math.floor(Math.random() * 100) < 6) {
			var smallestDistance = Infinity;
			var smallestDistanceIndex;
			for(var j=set1.length - 1; j>=0; --j) {
				d = distance(set2[i], set1[j]);
				if(d < smallestDistance) {
					smallestDistance = d;
					smallestDistanceIndex = j;
				}
				if(d < goodCorrespondenceThreshold) {
					break;
				}
			}
			if(smallestDistance < maximumPointMatchDistance) {
				indexPairs.push([set2[i], set1[smallestDistanceIndex]]);
			}
		}
	}

	return indexPairs;
}
function removeDuplicates(scan) {
	var allPoints = [];
	var remove = false;
	for(var i=0; i<scanRecord.length; ++i) {
		allPoints = allPoints.concat(scanRecord[i]);
	}
	for(var i=scan.length - 1; i>=0; --i) {
		remove = false;
		for(var j=0; j<allPoints.length; ++j) {
			var d = distance(scan[i], allPoints[j]);
			if(d < scanDensityDistance) {
				remove = true;
				break;
			}
		}
		if(remove) {
			scan.splice(i, 1);
		}
	}
	return scan;
}
function toggleScanning() {
	currentlyScanning = !currentlyScanning;
	overallCanvasDrag = [0, 0];
	lastOverallCanvasDrag = [0, 0];
	//This allows you to still zoom in and out on the map when the map is paused.
}
function manualFit(scan) {
	//Honestly, I don't even know if I'm going to do this.
	//I want to talk to someone else about the idea before I do it.
}
function mouseMoved(event) {
	currentMouseCoords[0] = event.clientX;
	currentMouseCoords[1] = event.clientY;
	//dataArea.innerHTML = "(" + currentMouseCoords[0] + ", " + currentMouseCoords[1] + ")<br>" + dataArea.innerHTML;
	if(wasTheCanvasClicked) {
		canvasDragged();
	}
}
function canvasClicked() {
	wasTheCanvasClicked = true;
	canvasClickedCoords = currentMouseCoords.slice(0);
}
function canvasReleased() {
	wasTheCanvasClicked = false;
	canvasClickedCoords = [0, 0];
	
	lastOverallCanvasDrag[0] = overallCanvasDrag[0];
	lastOverallCanvasDrag[1] = overallCanvasDrag[1];
}
function canvasDragged() {
	var canvasDragOffset = [];
	canvasDragOffset[0] = currentMouseCoords[0] - canvasClickedCoords[0];
	canvasDragOffset[1] = currentMouseCoords[1] - canvasClickedCoords[1];

	overallCanvasDrag[0] = (canvasDragOffset[0] / scaleFactor) + lastOverallCanvasDrag[0];
	overallCanvasDrag[1] = (-1 * canvasDragOffset[1] / scaleFactor) + lastOverallCanvasDrag[1];
}

//This actually sets it up so if you click "setup", the program starts.
startButton = document.getElementById("start");
startButton.addEventListener("click", setup);

//These are some of the html elements used.
enterZoomTextArea = document.getElementById("youSetZoom");
enterZoomButton = document.getElementById("enterZoom");
enterZoomButton.addEventListener("click", enterZoom);
document.getElementById("saveScan").addEventListener("click", saveScan);
document.getElementById("toggleScanning").addEventListener("click", toggleScanning);