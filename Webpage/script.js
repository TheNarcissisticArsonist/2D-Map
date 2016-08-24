//Constants
var webSocketIP = "127.0.0.1"; //The IP of the websocket server.
var webSocketPort = "12345"; //The port the websocket is being served on.
var webSocketPath = "/"; //The file path of the websocket.
var formattedDataStringExpectedArrayLength = 46; //The expected length of the formatted data string from formatRawMessage().
var formattedDataStringPositionIndeces = [10, 11, 12]; //The location of the xyz position data in the formatted data string.
var formattedDataStringQuaternionIndeces = [14, 15, 16, 17]; //The location of the xyzw quaternion data in the formatted data string.
var eulerAngleUsed = 0; //Due to some weirdness with the robot's orientation data, the Euler angle for the XY-plane could be anything. 0 is roll, 1 is pitch, 2 is yaw.
var robotMarkerRadius = 0.3; //The radius of the circle that marks the robot's location, in meters.
var robotMarkerArrowAngle = Math.PI/6; //There's an arrow on the circle, showing which direction the robot is pointing. This is the angle between the centerline and one of the sides.
var scaleFactorMultiplier = 50; //This lets it default to 1 pixel = 2 cm.
var distanceDisplayThreshold = 0.1; //If the distance between two points in a scan is greater than 0.1, it won't draw a line between them.
var distanceDisplayThresholdSquared = Math.pow(distanceDisplayThreshold, 2); //It's squared so it can be used with distanceSquared.
var scanThetaMinIndex = 37; //This is the formatted array index of the minimum angle for the scan.
var scanThetaMaxIndex = 38; //Same, but for the maximum angle.
var scanRangeListIndex = 44; //Same, but for the list of ranges.
var minimumPositionDistanceToRecord = 0.01; //If the distance between two position samples is less than this, only one of the points will be kept. This is in meters.
var icpAverageDistanceTraveledThreshold = 0.01; //The average distance traveled per point must be less than this for ICP to finish.
var icpAverageDistanceTraveledThresholdSquared = Math.pow(icpAverageDistanceTraveledThreshold, 2); //This is squared for use with the distanceSquared function.
var icpNoMovementCounterThreshold = 5; //ICP must lead to no movement at least this many times for it to finish.
var scanDensityDistance = 0.01; //In meters, the minimum significant distance between two points.
var scanDensityDistanceSquared = Math.pow(scanDensityDistance, 2); //For use with distanceSquared.
var maxICPLoopCount = 250; //The maximum number of times ICP can run.
var minICPComparePoints = 3000; //The minimum number of points ICP must use to compare.
var maximumPointMatchDistance = 2; //The maximum distance between matched points for ICP.
var goodCorrespondenceThreshold = 0; //If during point matching, the distance between two matched points is less than this, don't test any further points for a closer match.
var goodCorrespondenceThresholdSquared = Math.pow(goodCorrespondenceThreshold, 2); //If this is to be implemented, distanceSquared will be faster.
var maxNumFailedScans = 10; //How many scans have to fail in a row to go to user-manual fit.
var numRecentScans = 100; //How many scans to show when recentScansOnly is true.
var mapIncrementScanning = 1; //The value for mapIncrement used while scanning.
var mapIncrementPretty = 1; //The value for mapIncrement used while not scanning.
var zoomScrollConstant = 120*4; //How much a scroll is divided by when zooming in or out. This is really specific to which mouse you use, and your preferences.
var scansToSearchBackForDuplicates = 25; //How many scans are looked at when testing if duplicate points should be removed.
var loopClosureIterationPower = 1; //The power the iteration is raised to when computed in the stochastic gradient descent.
var loopClosureMaxIterations = 500; //The maximum number of iterations that loop closure will run.
var mapDataDisplayPageWidth = 500; //The width of the page displaying raw map data for the purposes of saving data.
var mapDataDisplayPageHeight = 500; //The same as above, but this time, the height.
var highlightedPoseCircleRadius = robotMarkerRadius / 10; //The radius of the circle that marks a highlighted pose.

//Global variables
var positionRecord = []; //This is the list of 2D points where the robot has been, so the program can draw lines between them.
var optimizedScanRecord = []; //This is the list of laser scans. The indeces correspond with positionRecord[]. They're in x-y position format, the same as positionRecord.
var scaleFactor = scaleFactorMultiplier; //As the path and information get bigger, it's useful to zoom out.
var currentlyScanning = true; //Used to know when to stop asking for more scans.
var lastAngle = 0; //Saved each iteration in case the user turns off scans. Updated in mainLoop.
var lastPosition = [0, 0]; //Saved each iteration, for the same reason as above. Updated in drawRobotPath.
var numFailedScans = 0; //How many scans have failed in a row.
var currentMouseCoords = [0, 0]; //The current coordinates of the mouse.
var canvasClickedCoords = [0, 0]; //The coordinates where the canvas was clicked.
var wasTheCanvasClicked = false; //Pretty self explanatory.
var overallCanvasDrag = [0, 0]; //This is applied to context transforms, so you can drag the map.
var lastOverallCanvasDrag = [0, 0]; //This is used so that when you drag the map, it's then applied to the next time you drag it.
var recentScansOnly = true; //When this is true, only recent scans will be shown, to speed up the program.
var mapIncrement = mapIncrementScanning; //What fraction of scans to display on the map.
var wasTheRotateClicked = false; //Whether the map rotate wheel was clicked or not.
var rotateClickedAngle = 0; //The angle where the rotation thing was clicked.
var lastOverallRotateDrag = 0; //This makes it so you can rotate the map.
var overallRotateDrag = 0; //Ditto the above.
var poses = []; //A list of poses to be used with loop closure.
var constraints = []; //A list of constraints between poses.
var angleOffset = 0; //Calculated with ICP to correct the robot's orientation.
var positionOffset = [0, 0]; //Ditto the above, but for position.
var rotationTransformOffset = [[1, 0], [0, 1]]; //A rotation matrix that is used for offsetting the position, just like above.
var highlightedPoses = []; //A list of poses to highlight on the map, to make scan matching easier.

//HTML Elements
var canvas, context, dataArea, updateZoomButton, enterZoomTextArea, enterZoomButton, autoZoomButton, startButton, outerCircle, highlightedScanTextArea, highlightedScanButton;
var unHighlightedScanTextArea, unHighlightedScanButton;

function setup() {
	console.log("Running setup function.");

	canvas = document.getElementById("mainCanvas"); //Grab the HTML canvas element.
	canvas.style.transform = "matrix(0, -1, 1, 0, 0, 0)"; //Rotate the canvas so up is forward, like in a map.
	context = canvas.getContext("2d"); //All canvas drawings are done through a context.
	context.fillStyle = "white"; //Set the fill style of closed shapes on the canvas to white.
	context.beginPath(); //This starts a path with which lines can be drawn.

	//Creating various event listeners for UI elements, such as clicking and dragging the canvas and such.
	canvas.addEventListener("mousedown", canvasClicked);
	canvas.addEventListener("mousewheel", function(event) {zoomed(event);});
	outerCircle = document.getElementById("outerCircle");
	outerCircle.addEventListener("mousedown", rotateClicked);
	document.body.addEventListener("mousemove", function(event) { mouseMoved(event); });
	document.body.addEventListener("mouseup", clickReleased);
	document.getElementById("toggleScanning").addEventListener("click", toggleScanning);
	document.getElementById("loopClosure").addEventListener("click", loopClosureButtonClicked);
	
	highlightedScanTextArea = document.getElementById("youHighlightScans");
	highlightedScanButton = document.getElementById("enterHighlightScans");
	highlightedScanButton.addEventListener("click", userScanHighlighted);
	unHighlightedScanTextArea = document.getElementById("youUnHighlightScans");
	unHighlightedScanButton = document.getElementById("enterUnHighlightScans");
	unHighlightedScanButton.addEventListener("click", userScanUnHighlighted);

	dataArea = document.getElementById("dataPrintout"); //As the program receives data, this area on the webpage can be used to record it.

	var webSocketAddress = "ws://"+webSocketIP+":"+webSocketPort+webSocketPath;
	ws = new WebSocket(webSocketAddress); //This creates the websocket object.
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
	context.rotate(overallRotateDrag);
	context.translate(overallCanvasDrag[1], -overallCanvasDrag[0]);
	setCanvasTransforms(robotPosition, robotOrientationTheta);
	drawRobotPath();
	drawRobotMap();
	drawHighlightedPoses();

	ws.send("Please do not reply."); //The server will see this and do nothing. This just keeps the websocket from timing out.

	requestAnimationFrame(function() { mainLoop(""); });
}
function normalMainLoop(formatted) {
	//console.log("Main loop!");
	//console.log(positionOffset);

	var robotPositionXYZ = getPositionFromFormattedMessage(formatted);
	var quaternion = getQuaternionFromFormattedMessage(formatted);
	var eulerAngles = quaternionToEuler(quaternion); //The robot sends orientation data in the form of a quaternion, whereas euler angles (the normal kind) are what canvas uses.

	//This offsets the position and orientation by the stored error.
	var robotOrientationTheta = eulerAngles[eulerAngleUsed] + angleOffset;
	var robotPosition = numeric.dot(rotationTransformOffset, robotPositionXYZ.slice(0, 2));
	robotPosition = numeric.add(robotPosition, positionOffset);

	lastAngle = robotOrientationTheta; //This is used if the user stops scanning.
	
	var scanDataFormatted = cleanUpScanData(formatted[scanThetaMinIndex], formatted[scanThetaMaxIndex], formatted[scanRangeListIndex]); //This converts the strings into useable numbers and arrays.
	var scanThetaMin = scanDataFormatted[0];
	var scanThetaMax = scanDataFormatted[1];
	var scanRangeList = scanDataFormatted[2];
	var scanAngleIncrement = (scanThetaMax - scanThetaMin) / scanRangeList.length; //This information is actually in the message, but I prefer to calculate it myself.
	
	var currentPose = [robotPosition[0], robotPosition[1], robotOrientationTheta];

	var currentPoseData = new pose(currentPose, scanThetaMin, scanThetaMax, scanAngleIncrement, scanRangeList);
	poses.push(currentPoseData);

	storePosition(robotPosition); //This appends the robotPosition to the positionRecord array, and does absolutely nothing else (yet).
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
	positionRecord.push(position);
	//This is its own function because I might later want to downsample, or do some other manipulation on the point.
}
function processScanData(angleMin, angleMax, rangeList, angleIncrement, robotPosition, robotAngle) {
	//Throughout this function, the scan is gradually refined from a list of distances to a set of newfound points to add to the map.
	var robotXYList = convertScanToRobotXY(angleMin, angleMax, rangeList, angleIncrement);
	var globalXYList = convertRobotXYToGlobalXY(robotXYList, robotPosition, robotAngle);
	var cleanGlobalXYList = removeScanNaN(globalXYList);

	if(optimizedScanRecord.length > 0) {
		//This is only run when there's at least one scan already stored, or there would be nothing for ICP to compare to!
		//if(numFailedScans < maxNumFailedScans) {
		cleanGlobalXYList = runICP(cleanGlobalXYList);
		var reducedGlobalXYList = removeDuplicates(cleanGlobalXYList);
		//}
		//else {
		//	manualFit(cleanGlobalXYList);
		//}
	}
	else {
		var reducedGlobalXYList = cleanGlobalXYList.slice(0);
	}

	if(reducedGlobalXYList.length > 0) {
		optimizedScanRecord.push(cleanGlobalXYList);
	}
}
function convertScanToRobotXY(min, max, rangeList, increment) {
	var currentTheta, x, y;
	var scan = [];
	for(var i=0; i<rangeList.length; ++i) {
		//This is basically converting a set of polar coordinates into cartesian coordinates, and then translating down 22 cm.
		//The downwards translation is because the laser scanner on the robot is ~22 cm in front of the center of mass, which is what odometry uses.
		currentTheta = min + (i * increment);
		x = rangeList[i] * Math.cos(currentTheta);
		y = rangeList[i] * Math.sin(currentTheta) * -1;
		x += 0.22;
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
	return Math.sqrt(distanceSquared(pointA, pointB));
}
function distanceSquared(pointA, pointB) {
	//This is the distance formula without the square root. When simply comparing to a constant, this is faster, as you can just square the constant.
	return Math.pow(pointB[0]-pointA[0], 2) + Math.pow(pointB[1]-pointA[1], 2);
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
	context.moveTo(positionRecord[0][0], positionRecord[0][1]);
	for(var i=1; i<positionRecord.length; ++i) {
		context.lineTo(positionRecord[i][0], positionRecord[i][1]);
	}
	var lastIndex = positionRecord.length - 1;
	lastPosition = [positionRecord[lastIndex][0], positionRecord[lastIndex][1]];
	context.stroke();
	context.strokeStyle = "#000000";
	context.beginPath();
}
function drawRobotMap() {
	context.strokeStyle = "#aa0000";
	context.beginPath();
	var startIndex = 0;
	if(recentScansOnly) {
		startIndex = (optimizedScanRecord.length - numRecentScans) - (optimizedScanRecord.length % mapIncrement);
	}
	for(var i=startIndex; i<optimizedScanRecord.length; i+=mapIncrement) {
		if(i >= 0) {
			scan = optimizedScanRecord[i];
			context.moveTo(scan[0][0], scan[0][1]);
			context.beginPath();
			for(var j=1; j<scan.length; ++j) {
				if(distanceSquared([scan[j][0], scan[j][1]], [scan[j-1][0], scan[j-1][1]]) > distanceDisplayThresholdSquared) {
					context.stroke();
					context.moveTo(scan[j][0], scan[j][1]);
					context.beginPath();
				}
				else {
					context.lineTo(scan[j][0], scan[j][1]);
				}
			}
		}
	}
	context.stroke();
	context.strokeStyle = "#000000";
	context.beginPath();
}
function runICP(scan) {
	var currentScan = [];
	var knownPoints = [];
	var finished = false;
	var iterationTotalSquaredDistance = 0;
	var iterationAverageSquaredDistance = 0;
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
	var scanTransformError = [[1, 0], [0, 1]];
	var i = optimizedScanRecord.length - 1;

	currentScan = scan.slice(0);

	while(i >= 0 && knownPoints.length <= minICPComparePoints) {
		knownPoints = knownPoints.concat(optimizedScanRecord[i]);
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
			scanTransformError = [[1, 0], [0, 1]];
			break;
		}
		iterationTotalSquaredDistance = 0;
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
			iterationTotalSquaredDistance += distanceSquared(oldScanPoints[i], currentScan[i]);
		}
		iterationAverageSquaredDistance = iterationTotalSquaredDistance / currentScan.length;

		if(iterationAverageSquaredDistance < icpAverageDistanceTraveledThresholdSquared) {
			++icpLoopCounter;
		//	console.log("Good scan! " + iterationAverageDistance);
			if(icpLoopCounter >= icpNoMovementCounterThreshold) {
				finished = true;
				console.log("Success! " + totalLoopCount);
				numFailedScans = 0;
			}
		}
		else {
			icpLoopCounter = 0;
		//	console.log("           " + iterationAverageDistance);
		}
		
		scanAngleError += angle;
		scanPositionError = numeric.add(scanPositionError, translation);
		scanTransformError = numeric.dot(scanTransformError, rotationMatrix);
	}

	//console.log("Angle error: " + scanAngleError);
	//console.log("Position error: " + scanPositionError[0] + ", " + scanPositionError[1]);

	angleOffset += scanAngleError;
	rotationTransformOffset = numeric.dot(rotationTransformOffset, scanTransformError);
	positionOffset = numeric.add(positionOffset, scanPositionError);

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
		if(Math.floor(Math.random() * 100) < 10) {
			var smallestSquaredDistance = Infinity;
			var smallestSquaredDistanceIndex;
			for(var j=set1.length - 1; j>=0; --j) {
				d = distanceSquared(set2[i], set1[j]);
				if(d < smallestSquaredDistance) {
					smallestSquaredDistance = d;
					smallestSquaredDistanceIndex = j;
				}
				if(d < goodCorrespondenceThresholdSquared) {
					break;
				}
			}
			if(smallestSquaredDistance < maximumPointMatchDistance) {
				indexPairs.push([set2[i], set1[smallestSquaredDistanceIndex]]);
			}
		}
	}

	return indexPairs;
}
function removeDuplicates(scan) {
	var allScans = [];
	var lowIndex = optimizedScanRecord.length - scansToSearchBackForDuplicates;
	var highIndex = optimizedScanRecord.length - 1; //Inclusive
	for(var i=lowIndex; i<=highIndex; ++i) {
		if(i >= 0) {
			allScans.push(optimizedScanRecord[i]);
		}
		else {
			++lowIndex;
		}
	}
	var remove = false;
	for(var i=scan.length - 1; i>=0; --i) {
		remove = false;
		var j, k;
		for(j=allScans.length - 1; j>=0; --j) {
			for(k=allScans[j].length - 1; k>=0; --k) {
				var d = distanceSquared(scan[i], allScans[j][k]);
				if(d < scanDensityDistanceSquared) {
					optimizedScanRecord[j + lowIndex].splice(k, 1);
					if(optimizedScanRecord[j].length == 0) {
						optimizedScanRecord.splice(j + lowIndex, 1);
					}
					remove = true;
					break;
				}
			}
			if(remove) {
				break;
			}
		}
	}
	return scan;
}
function toggleScanning() {
	//This allows you to still zoom in and out on the map when the map is paused.
	currentlyScanning = !currentlyScanning;
	overallCanvasDrag = [0, 0];
	lastOverallCanvasDrag = [0, 0];
	overallRotateDrag = 0;
	lastOverallRotateDrag = 0;

	//This makes sure all scans are displayed when not scanning, and a reduced number are displayed when scanning is happening, to make the program faster.
	recentScansOnly = currentlyScanning;
	if(currentlyScanning) {
		mapIncrement = mapIncrementScanning;
	}
	else {
		mapIncrement = mapIncrementPretty;
	}
}
function manualFit(scan) {
	//Honestly, I don't even know if I'm going to do this.
	//I want to talk to someone else about the idea before I do it.
}
function mouseMoved(event) {
	currentMouseCoords[0] = event.clientX;
	currentMouseCoords[1] = window.innerHeight - event.clientY;
	//dataArea.innerHTML = "(" + currentMouseCoords[0] + ", " + currentMouseCoords[1] + ")<br>" + dataArea.innerHTML;
	if(wasTheCanvasClicked) {
		canvasDragged();
	}
	else if(wasTheRotateClicked) {
		rotateDragged();
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

	var transformMatrix = [[Math.cos(-overallRotateDrag), -Math.sin(-overallRotateDrag)], [Math.sin(-overallRotateDrag), Math.cos(-overallRotateDrag)]];
	canvasDragOffset = numeric.dot(transformMatrix, canvasDragOffset);

	overallCanvasDrag[0] = (canvasDragOffset[0] / scaleFactor) + lastOverallCanvasDrag[0];
	overallCanvasDrag[1] = (canvasDragOffset[1] / scaleFactor) + lastOverallCanvasDrag[1];
}
function rotateClicked() {
	var circleBoundingBox = outerCircle.getBoundingClientRect();
	circleCenterCoords = [circleBoundingBox.left + (outerCircle.r.baseVal.value/2), window.innerHeight - circleBoundingBox.top - (outerCircle.r.baseVal.value/2)];
	wasTheRotateClicked = true;
	rotateClickedAngle = Math.PI + Math.atan2(currentMouseCoords[1] - circleCenterCoords[1], currentMouseCoords[0] - circleCenterCoords[0]);
}
function rotateReleased() {
	wasTheRotateClicked = false;
	rotateClickedAngle = 0;

	lastOverallRotateDrag = overallRotateDrag;
}
function rotateDragged() {
	var circleBoundingBox = outerCircle.getBoundingClientRect();
	circleCenterCoords = [circleBoundingBox.left + (outerCircle.r.baseVal.value/2), window.innerHeight - circleBoundingBox.top - (outerCircle.r.baseVal.value/2)];
	var newTheta = Math.PI + Math.atan2(currentMouseCoords[1] - circleCenterCoords[1], currentMouseCoords[0] - circleCenterCoords[0]);
	var thetaOffset = newTheta - rotateClickedAngle;
	overallRotateDrag = thetaOffset + lastOverallRotateDrag;
}
function clickReleased() {
	if(wasTheCanvasClicked) {
		canvasReleased();
	}
	else if(wasTheRotateClicked) {
		rotateReleased();
	}
}
function zoomed(e) {
	if(optimizedScanRecord.length == 0) {
		//Don't zoom if there's nothing to see.
		return;
	}

	var zoomMultiplier = Math.pow(2, e.wheelDelta/zoomScrollConstant); //Raising 2 to the power of wheelDelta changes it from a positive/negative number to a number that is greater than or less than 1, and so it's fit for a scale factor.
	scaleFactor *= zoomMultiplier;
}
function runLoopClosure() {
	var iteration = 0;
	var finished = false;
	var M = [];
	var oldLastPose = poses[poses.length - 1];
	
	while(!finished) {
		++iteration;
		if(iteration > loopClosureMaxIterations) {
			finished = true;
		}
		else {
			var gamma = [Infinity, Infinity, Infinity];

			for(var i=0; i<poses.length; ++i) {
				M.push([]);
				for(var j=0; j<3; ++j) {
					M[i][j] = 0;
				}
			}

			for(var i=0; i<constraints.length; ++i) {
				var theta = poses[constraints[i].a].pose[2];
				var R = [
					[Math.cos(theta), -Math.sin(theta), 0],
					[Math.sin(theta), Math.cos(theta), 0],
					[0, 0, 1]
				];
				var W = numeric.inv(numeric.dot(numeric.dot(R, constraints[i].sigma), numeric.transpose(R)));

				for(var j=constraints[i].a + 1; j<=constraints[i].b; ++j) {
					M[j][0] += W[0][0];
					M[j][1] += W[1][1];
					M[j][2] += W[2][2];

					if((Math.pow(gamma[0], 2) + Math.pow(gamma[1], 2) + Math.pow(gamma[2], 2)) > (Math.pow(W[0][0], 2) + Math.pow(W[1][1], 2) + Math.pow(W[2][2], 2))) {
						gamma[0] = W[0][0];
						gamma[1] = W[1][1];
						gamma[2] = W[2][2];
					}
				}
			}
			for(var i=0; i<constraints.length; ++i) {
				var theta = poses[constraints[i].a].pose[2]
				
				var Pa = [
					[Math.cos(theta), -Math.sin(theta), poses[constraints[i].a].pose[0]],
					[Math.sin(theta), Math.cos(theta), poses[constraints[i].a].pose[1]],
					[0, 0, 1]
				];

				var R = [
					[Pa[0][0], Pa[0][1], 0],
					[Pa[1][0], Pa[1][1], 0],
					[0, 0, 1]
				];

				var Tab = [
					[Math.cos(constraints[i].t[2]), -Math.sin(constraints[i].t[2]), constraints[i].t[0]],
					[Math.sin(constraints[i].t[2]), Math.cos(constraints[i].t[2]), constraints[i].t[1]],
					[0, 0, 1]
				];

				var PbPrime = numeric.dot(Pa, Tab);

				var angleInformationVector = numeric.dot(PbPrime, [1, 0, 0]);
				var angle = Math.atan2(angleInformationVector[1], angleInformationVector[0]);

				var translationInformationVector = numeric.dot(PbPrime, [0, 0, 1]);

				var r = [
					translationInformationVector[0] - poses[constraints[i].b].pose[0],
					translationInformationVector[1] - poses[constraints[i].b].pose[1],
					angle - poses[constraints[i].b].pose[2]
				];

				while(r[2] > 2 * Math.PI) { r[2] -= 2 * Math.PI; }
				while(r[2] < 0) { r[2] += 2 * Math.PI; }

				var d = numeric.dot(2, numeric.dot(numeric.inv(numeric.dot(numeric.dot(numeric.transpose(R), constraints[i].sigma), R)), r));

				for(var j=0; j<3; ++j) {
					var alpha = 1/(gamma[j] * Math.pow(iteration, loopClosureIterationPower));

					var totalWeight = 0;
					for(var k=constraints[i].a+1; k<=constraints[i].b; ++k) {
						totalWeight += 1/M[k][j];
					}

					var beta = (constraints[i].b - constraints[i].a) * d[j] * alpha;
					if(Math.abs(beta) > Math.abs(r[j])) {
						beta = r[j];
					}

					var dPose = 0;
					for(var k=constraints[i].a+1; k<poses.length; ++k) {
						if(k >= constraints[i].a+1 && k <= constraints[i].b) {
							dPose += beta/M[k][j]/totalWeight;
						}
						poses[k].pose[j] += dPose;
					}
				}
			}
		}
	}

	updateSLAM(oldLastPose);
	console.log(optimizedScanRecord);
	deleteOldMap();
	recalculateMapFromPoses(0);
	console.log(optimizedScanRecord);
}
function updateSLAM(oldPose) {
	var newPose = poses[poses.length - 1];

	var thetaDifference = newPose.pose[2] - oldPose.pose[2];
	var positionDifference = [0, 0];
	positionDifference[0] = newPose.pose[0] - oldPose.pose[0];
	positionDifference[1] = newPose.pose[1] - oldPose.pose[1];
	var rotationMatrix = [[Math.cos(thetaDifference), -Math.sin(thetaDifference)], [Math.sin(thetaDifference), Math.cos(thetaDifference)]];

	angleOffset += angleOffset;
	rotationTransformOffset = numeric.dot(rotationTransformOffset, rotationMatrix);
	positionOffset = numeric.add(positionDifference, positionOffset);
}
function deleteOldMap() {
	positionRecord = [];
	optimizedScanRecord = [];
}
function recalculateMapFromPoses(iteration) {
	var i = iteration;
	var position = [];
	position[0] = poses[i].pose[0];
	position[1] = poses[i].pose[1];
	var robotTheta = poses[i].pose[2];
	var thetaMin = poses[i].scanMinTheta;
	var thetaMax = poses[i].scanMaxTheta;
	var rangeList = poses[i].scanRangeList.slice(0);
	var thetaIncrement = (thetaMax - thetaMin) / rangeList.length;
	
	storePosition(position);

	var robotXYList = convertScanToRobotXY(thetaMin, thetaMax, rangeList, thetaIncrement);
	var globalXYList = convertRobotXYToGlobalXY(robotXYList, position, robotTheta);
	var cleanGlobalXYList = removeScanNaN(globalXYList);
	var reducedGlobalXYList = removeDuplicates(cleanGlobalXYList);

	if(reducedGlobalXYList.length > 0) {
		optimizedScanRecord.push(cleanGlobalXYList);
	}

	++i;

	if(i < poses.length) {
		//window.setTimeout(function() {
			recalculateMapFromPoses(i);
		//}, 0);
	}
}
function loopClosureButtonClicked() {
	if(!currentlyScanning) {
		console.log("Running loop closure!");
		runLoopClosure();
	}
}
function saveMap() {
	var mapData = {};
	mapData.poseList = poses.slice(0);

	var mapDataString = JSON.stringify(mapData, null, 4);

	var jsonDataWindow = window.open("", "", "width=" + mapDataDisplayPageWidth + ", height=" + mapDataDisplayPageHeight + "");
	requestAnimationFrame(function() { jsonDataWindow.document.body.innerHTML = "<pre>" + mapDataString + "</pre>"; });
}
function drawHighlightedPoses() {
	context.strokeStyle = "#0000ff";
	for(var i=0; i<highlightedPoses.length; ++i) {
		var pose = poses[highlightedPoses[i]];
		context.beginPath();
		context.arc(pose.pose[0], pose.pose[1], highlightedPoseCircleRadius, 0, 2 * Math.PI);
		context.lineTo(pose.pose[0], pose.pose[1]);
		context.stroke();
	}
	context.strokeStyle = "#000000";
}
function userScanHighlighted() {
	rawIndex = Number(highlightedScanTextArea.value);
	if(!isNaN(rawIndex)) { //Make sure it's a number.
		if(rawIndex >= 0 && rawIndex < poses.length) { //Make sure it's a proper index.
			highlightedPoses.push(rawIndex);
		}
	}
}
function userScanUnHighlighted() {
	rawIndex = Number(unHighlightedScanTextArea.value);
	if(!isNaN(rawIndex)) { //Make sure it's a number.
		if(rawIndex >= 0 && rawIndex < poses.length) { //Make sure it's a proper index.
			var metaIndex = highlightedPoses.indexOf(rawIndex);
			highlightedPoses.splice(metaIndex, 1);
		}
	}
}

function pose(pose, scanMinTheta, scanMaxTheta, scanThetaIncrement, scanRangeList) {
	this.pose = pose;
	this.scanMinTheta = scanMinTheta;
	this.scanMaxTheta = scanMaxTheta;
	this.scanThetaIncrement = scanThetaIncrement;
	this.scanRangeList = scanRangeList;
}
function constraint(a, b, t, sigma) {
	this.a = a;
	this.b = b;
	this.t = t;
	this.sigma = sigma;
}

//This actually sets it up so if you click "setup", the program starts.
startButton = document.getElementById("start");
startButton.addEventListener("click", setup);

//These are some of the html elements used.
enterZoomTextArea = document.getElementById("youSetZoom");
enterZoomButton = document.getElementById("enterZoom");
enterZoomButton.addEventListener("click", enterZoom);