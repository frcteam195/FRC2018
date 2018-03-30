var waypoints = [];
var arcArr = [];
var ctx;
var width = 1656; //pixels
var height = 823; //pixels
var fieldWidth = 652; // in inches
var fieldHeight = 324; // in inches
var robotWidth = 32.75; //inches
var robotHeight = 37.5; //inches
// var robotWidth = 25.5; //inches wheelbase
// var robotHeight = 26.377; //inches wheelbase
var pointRadius = 5;
var turnRadius = 30;
var kEpsilon = 1E-9;
var image;
var imageFlipped;
var wto;

var startLeftY = 276;
var startRightY = 48;

var maxSpeed = 120;
var maxSpeedColor = [0, 255, 0];
var minSpeed = 0;
var minSpeedColor = [255, 0, 0];
var pathFillColor = "rgba(150, 150, 150, 0.5)";

const wheelDiameter = 5;	//Wheel Diameter in inches
const accelValue = 10; 	//Inches/second^2
const maxVelocity = 20; //Inches/second
const timeStep = 0.01;	//Time step in seconds
const encoderTicksPerRev = 4096;

class Translation2d {
	constructor(x, y) {
		this.x = x;
		this.y = y;
	}

	norm() {
		return Math.sqrt(Translation2d.dot(this, this));
	}

	scale(s) {
		return new Translation2d(this.x * s, this.y * s);
	}

	translate(t) {
		return new Translation2d(this.x + t.x, this.y + t.y);
	}

	invert() {
		return new Translation2d(-this.x, -this.y);
	}

	perp() {
		return new Translation2d(-this.y, this.x);
	}

	draw(color) {
		color = color || "#f72c1c";
		ctx.beginPath();
		ctx.arc(this.drawX, this.drawY, pointRadius, 0, 2 * Math.PI, false);
		ctx.fillStyle = color;
		ctx.strokeStyle = color;
		ctx.fill();
		ctx.lineWidth = 0;
		ctx.stroke();
	}

	get drawX() {
		return this.x*(width/fieldWidth);
	}

	get drawY() {
		return height - this.y*(height/fieldHeight);
	}

	get angle() {
		return Math.atan2(-this.y, this.x);
	}

	static diff(a, b) {
		return new Translation2d(b.x - a.x, b.y - a.y);
	}

	static cross(a, b) {
		return a.x * b.y - a.y * b.x;
	}

	static dot(a, b) {
		return a.x * b.x + a.y * b.y;
	}

	static angle(a, b) {
		return Math.acos(Translation2d.dot(a,b) / (a.norm() * b.norm()));
	}
}

class Waypoint {
	constructor(position, speed, radius, marker, comment) {
		this.position = position;
		this.speed = speed;
		this.radius = radius;
		this.marker = marker;
		this.comment = comment;
	}

	draw() {
		this.position.draw((this.radius > 0) ? "rgba(120,120,120,0.8)" : null);
	}

	toString() {
		if (this.marker === "")
			return "new Waypoint("+this.position.x+","+this.position.y+","+this.radius+","+this.speed+")";
		else
            return "new Waypoint("+this.position.x+","+this.position.y+","+this.radius+","+this.speed+",\""+this.marker.trim()+"\")";
	}
}

class Line {
	constructor(pointA, pointB) {
		this.pointA = pointA;
		this.pointB = pointB;
		this.slope = Translation2d.diff(pointA.position, pointB.position);
		this.start = pointA.position.translate( this.slope.scale( pointA.radius/this.slope.norm() ) );
		this.end = pointB.position.translate( this.slope.scale( pointB.radius/this.slope.norm() ).invert() );
	}

	draw() {
		ctx.beginPath();
        ctx.moveTo(this.start.drawX, this.start.drawY);
        ctx.lineTo(this.end.drawX, this.end.drawY);
        
		try {
        	var grad = ctx.createLinearGradient(this.start.drawX, this.start.drawY, this.end.drawX, this.end.drawY);
	grad.addColorStop(0, getColorForSpeed(this.pointB.speed));
		grad.addColorStop(1, getColorForSpeed(getNextSpeed(this.pointB)));
		ctx.strokeStyle = grad;
		} catch (e) {
			ctx.strokeStyle = "#00ff00"
		}

        ctx.lineWidth = pointRadius * 2;
        ctx.stroke();
        this.pointA.draw();
        this.pointB.draw();
	}

	fill() {
		var start = this.start;
		var deltaEnd = Translation2d.diff(this.start,this.end);
		var angle = deltaEnd.angle;
		var length = deltaEnd.norm();
		for(var i=0; i<length; i++) {
		drawRotatedRect(start.translate(deltaEnd.scale(i/length)), robotHeight, robotWidth, angle, null, pathFillColor, true);
		}
	}

	length() {
        return Math.sqrt(Math.pow(this.end.x-this.start.x, 2) + Math.pow(this.end.y-this.start.y, 2));
	}

	translation() {
		return new Translation2d(this.pointB.position.y - this.pointA.position.y, this.pointB.position.x - this.pointA.position.x)
	}

	slope() {
		if(this.pointB.position.x - this.pointA.position.x > kEpsilon)
			return (this.pointB.position.y - this.pointA.position.y) / (this.pointB.position.x - this.pointA.position.x);
		else
			return (this.pointB.position.y - this.pointA.position.y) / kEpsilon;
	}

	b() {
		return this.pointA.y - this.slope() * this.pointA.x;
	}

	static intersect(a, b, c, d) {
		var i = ((a.x-b.x)*(c.y-d.y) - (a.y-b.y)*(c.x-d.x));
		i = (Math.abs(i) < kEpsilon) ? kEpsilon : i;
		var x = (Translation2d.cross(a, b) * (c.x - d.x) - Translation2d.cross(c, d)*(a.x - b.x)) / i;
		var y = (Translation2d.cross(a, b) * (c.y - d.y) - Translation2d.cross(c, d)*(a.y - b.y)) / i;
		return new Translation2d(x, y);
	}

	static pointSlope(p, s) {
		return new Line(p, p.translate(s));
	}
}

class Arc {
	constructor(lineA, lineB) {
		this.lineA = lineA;
		this.lineB = lineB;
		this.center = Line.intersect(lineA.end, lineA.end.translate(lineA.slope.perp()), lineB.start, lineB.start.translate(lineB.slope.perp()));
		this.center.draw;
		this.radius = Translation2d.diff(lineA.end, this.center).norm();
	}

	draw() {
		var sTrans = Translation2d.diff(this.center, this.lineA.end);
		var eTrans = Translation2d.diff(this.center, this.lineB.start);
		//console.log(sTrans);
		//console.log(eTrans);
		var sAngle, eAngle;
		if(Translation2d.cross(sTrans, eTrans) > 0) {
			eAngle = -Math.atan2(sTrans.y, sTrans.x);
			sAngle = -Math.atan2(eTrans.y, eTrans.x);
		} else {
			sAngle = -Math.atan2(sTrans.y, sTrans.x);
			eAngle = -Math.atan2(eTrans.y, eTrans.x);
		}
		this.lineA.draw();
		this.lineB.draw();
		ctx.beginPath();
		ctx.arc(this.center.drawX,this.center.drawY,this.radius*(width/fieldWidth),sAngle,eAngle);
		ctx.strokeStyle=getColorForSpeed(this.lineB.pointB.speed);
		ctx.stroke();
	}

	fill() {
		this.lineA.fill();
		this.lineB.fill();
		var sTrans = Translation2d.diff(this.center, this.lineA.end);
		var eTrans = Translation2d.diff(this.center, this.lineB.start);
		var sAngle = (Translation2d.cross(sTrans, eTrans) > 0) ? sTrans.angle : eTrans.angle;
		var angle = Translation2d.angle(sTrans, eTrans);
		var length = angle * this.radius;
		for(var i=0; i<length; i+=this.radius/100) {
		drawRotatedRect(this.center.translate(new Translation2d(this.radius*Math.cos(sAngle-i/length*angle),-this.radius*Math.sin(sAngle-i/length*angle))), robotHeight, robotWidth, sAngle-i/length*angle+Math.PI/2, null, pathFillColor, true);
		}

		

	}

	arcLength() {
		if (typeof this.lineA !== 'undefined' && typeof this.lineB !== 'undefined') {
            return 2 * this.radius * Math.asin(this.pointDistance(this.lineA.end.x, this.lineA.end.y, this.lineB.start.x, this.lineB.start.y) / (2 * this.radius));
        }
		else
			console.log("Error calculating length");
	}

	getPointsFromArc(initialPosition, initialVelocity) {
		var points = [];
        var posCurrent = initialPosition;
		//TODO: Fix overlapping segment issue
        var i = 0;
        var lineALength = this.lineA.length() + posCurrent;
        console.log("Line A Length: " + lineALength);
        for (; posCurrent < lineALength; i++) {
            var tCurrent = i * timeStep;
            var currentAccel = 0;

            currentAccel = velCurrent >= maxVelocity ? 0 : accelValue;

            posCurrent = Math.min(currentAccel * Math.pow(tCurrent,2) * 0.5 + initialVelocity * tCurrent + initialPosition, lineALength);
            var velCurrent = currentAccel * tCurrent + initialVelocity;
            velCurrent = velCurrent > maxVelocity ? maxVelocity : velCurrent;
            //velCurrent = velCurrent < 0 ? 0 : velCurrent;
            points.push(new TalonSRXPoint(convertInchestoRotations(posCurrent), convertInchesPerSecondToNativeUnitsPer100ms(velCurrent), timeStep));
            initialVelocity = velCurrent;
            initialPosition = posCurrent;
		}

		if (!((this.lineA.start.x === this.lineA.end.x || this.lineB.start.x === this.lineB.end.x) || (this.lineA.start.y === this.lineA.end.y || this.lineB.start.y === this.lineB.end.y))) {
            var currArcLength = this.arcLength() + posCurrent;
            console.log("Arc Length: " + currArcLength);
            for (; posCurrent < currArcLength; i++) {
                var tCurrent = i * timeStep;
                var currentAccel = 0;

                currentAccel = velCurrent >= maxVelocity ? 0 : accelValue;

                posCurrent = Math.min(currentAccel * Math.pow(tCurrent, 2) * 0.5 + initialVelocity * tCurrent + initialPosition, currArcLength);
                var velCurrent = currentAccel * tCurrent + initialVelocity;
                velCurrent = velCurrent > maxVelocity ? maxVelocity : velCurrent;
                //velCurrent = velCurrent < 0 ? 0 : velCurrent;
                points.push(new TalonSRXPoint(convertInchestoRotations(posCurrent), convertInchesPerSecondToNativeUnitsPer100ms(velCurrent), timeStep));
                initialVelocity = velCurrent;
                initialPosition = posCurrent;
            }
        }

        var lineBLength = this.lineB.length() + posCurrent;
        console.log("Line B Length: " + lineBLength);
        for (; posCurrent < lineBLength; i++) {
            var tCurrent = i * timeStep;
            var currentAccel = 0;

            currentAccel = velCurrent >= maxVelocity ? 0 : accelValue;

            posCurrent = Math.min(currentAccel * Math.pow(tCurrent,2) * 0.5 + initialVelocity * tCurrent + initialPosition, lineBLength);
            var velCurrent = currentAccel * tCurrent + initialVelocity;
            velCurrent = velCurrent > maxVelocity ? maxVelocity : velCurrent;
            //velCurrent = velCurrent < 0 ? 0 : velCurrent;
            points.push(new TalonSRXPoint(convertInchestoRotations(posCurrent), convertInchesPerSecondToNativeUnitsPer100ms(velCurrent), timeStep));
            initialVelocity = velCurrent;
            initialPosition = posCurrent;
        }

		return points;
	}

    pointDistance(x1, y1, x2, y2) {
        return Math.sqrt(Math.pow(x2-x1, 2) + Math.pow(y2-y1, 2));
    }

	static fromPoints(a, b, c) {
		return new Arc( new Line(a, b), new Line(b, c));
	}
}

class TalonSRXPoint {
	constructor (pos, vel, time) {
		this.pos = pos;
		this.vel = vel;
		this.time = time;
	}
}

function convertInchestoRotations(inches) {
    return inches / (wheelDiameter * Math.PI);
}

function convertInchesPerSecondToNativeUnitsPer100ms(ips) {
	return (ips * 60) / (wheelDiameter * Math.PI) * encoderTicksPerRev / 600;
}

function convertRotationstoInches(rotations) {
    return rotations * (wheelDiameter * Math.PI);
}

function convertNativeUnitsPer100msToInchesPerSecond(ips) {
    return (ips / 60) * (wheelDiameter * Math.PI) / encoderTicksPerRev * 600;
}

function init() { 
	$("#field").css("width", (width / 1.5) + "px");
	$("#field").css("height", (height / 1.5) + "px");
	ctx = document.getElementById('field').getContext('2d')
    ctx.canvas.width = width;
    ctx.canvas.height = height;
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle="#FF0000";
    image = new Image();
    image.src = 'files/field.png';
    image.onload = function(){
        ctx.drawImage(image, 0, 0, width, height);
        update();
    }
    //imageFlipped = new Image();
    //imageFlipped.src = 'fieldflipped.png';
    $('input').bind("change paste keyup", function() {
		console.log("change");
		clearTimeout(wto);
			wto = setTimeout(function() {
			update();
		}, 500);
	});
}

function clear() {
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle="#FF0000";
    if(flipped)
    	ctx.drawImage(imageFlipped, 0, 0, width, height);
    else
    	ctx.drawImage(image, 0, 0, width, height);
}

var f;
// function create() {
// 	var a = new Waypoint(new Translation2d(30,30), 0,0,0)
// 	var b = new Waypoint(new Translation2d(230,30), 0,30,0)
// 	var c = new Waypoint(new Translation2d(230,230), 0,0,0)
// 	var d = new Line(a, b);
// 	var e = new Line(b, c);
// 	f = new Arc(d, e);
// }

function addPoint(x, y, radius) {
	var prev;
	if(waypoints.length > 0)
		prev = waypoints[waypoints.length - 1].position;
	else 
		prev = new Translation2d(20, 260);
		
  	if(typeof x === "undefined") {x = prev.x + 20;}
  	
	if(typeof y === "undefined") {y = prev.y;}

    if(typeof radius === "undefined") {
  		radius = 0;
  		if ($("table tr").length > 1 && waypoints.length > 1) {
            if (x !== waypoints[waypoints.length - 1].position.x && x !== waypoints[waypoints.length - 2].position.x &&
				y !== waypoints[waypoints.length - 1].position.y && y !== waypoints[waypoints.length - 2].position.y) {
                var idx = parseInt($('tbody').children().length) - 1;
                $($($($('tbody').children()[idx]).children()[2]).children()).val(15);
            }
		}
  	}

    x = Math.round(x);
    y = Math.round(y);
    radius = Math.round(radius);
	
	$("tbody").append("<tr>"
		+"<td><input value='"+(x)+"'></td>"
		+"<td><input value='"+(y)+"'></td>"
		+"<td><input value='"+(radius)+"'></td>"
		+"<td><input value='60'></td>"
		+"<td class='marker'><input placeholder='Marker'></td>"
		+"<td><button onclick='$(this).parent().parent().remove();update();'>Delete</button></td></tr>"
	);
	update();
	$('input').unbind("change paste keyup");
	$('input').bind("change paste keyup", function() {
		console.log("change");
		clearTimeout(wto);
			wto = setTimeout(function() {
			update();
		}, 500);
	});
}

function update() {
    if ($("table tr").length > 1) {
        var idx = parseInt($('tbody').children().length) - 1;
    	if ($($($($('tbody').children()[idx]).children()[2]).children()).val() !== 0) {
            $($($($('tbody').children()[idx]).children()[2]).children()).val(0);
        }
    }

	waypoints = [];
	$('tbody').children('tr').each(function () {
        var x = parseInt( $($($(this).children()).children()[0]).val() );
        //console.log(x);
        var y = parseInt( $($($(this).children()).children()[1]).val() );
        var radius = parseInt( $($($(this).children()).children()[2]).val() );
        var speed = parseInt( $($($(this).children()).children()[3]).val() );
        if(isNaN(radius) || isNaN(speed)) {
        	radius = 0;
        	speed = 0;
        }
        var marker = ( $($($(this).children()).children()[4]).val() )
        var comment = ( $($($(this).children()).children()[5]).val() )
        waypoints.push(new Waypoint(new Translation2d(x,y), speed, radius, marker, comment));
    });
    drawPoints();
    drawRobot();
}

function drawRobot() {
	if(waypoints.length > 1) {
		var deltaStart = Translation2d.diff(waypoints[0].position, waypoints[1].position);
		drawRotatedRect(waypoints[0].position, robotHeight, robotWidth, deltaStart.angle, getColorForSpeed(waypoints[1].speed));

		var deltaEnd = Translation2d.diff(waypoints[waypoints.length-2].position, waypoints[waypoints.length-1].position);
		drawRotatedRect(waypoints[waypoints.length-1].position, robotHeight, robotWidth, deltaEnd.angle, getColorForSpeed(0));
	}
}

function drawRotatedRect(pos,w,h,angle,strokeColor,fillColor,noFill){
	w = w*(width/fieldWidth);
	h = h*(height/fieldHeight);
	fillColor = fillColor || "rgba(0,0,0,0)";
	//ctx.save();
	if(noFill == null || !noFill)
		ctx.beginPath();
	ctx.translate(pos.drawX, pos.drawY);
	ctx.rotate(angle);
    	ctx.rect(-w/2, -h/2, w,h);
	ctx.fillStyle = fillColor;
	if(noFill == null || !noFill)
		ctx.fill();
	if(strokeColor != null) {
		ctx.strokeStyle = strokeColor;
		ctx.lineWidth = 4;
		ctx.stroke();
	}
	ctx.rotate(-angle);
	ctx.translate(-pos.drawX, -pos.drawY);
	//ctx.restore();

}

function drawPoints() {
	clear();
	arcArr = [];
	var i = 0;
	ctx.beginPath();
	do {
		var a = Arc.fromPoints(getPoint(i), getPoint(i+1), getPoint(i+2));
		a.fill();
		i++;
	} while(i < waypoints.length - 2);
	ctx.fill();
	i=0;
	do {
		var a = Arc.fromPoints(getPoint(i), getPoint(i+1), getPoint(i+2));
		arcArr.push(a);
		a.draw();
		i++;
	} while(i < waypoints.length - 2);

}

function doStuff() {
	var points = [];
	console.log(arcArr);
	for (var i = 0; i < arcArr.length; i++) {
		console.log(arcArr[i]);

         var tmp = arcArr[i];
        if (i === 0) {
            console.log("Beginning zero arc");
			var tmpPoints = tmp.getPointsFromArc(0, 0);

			for (var j = 0; j < tmpPoints.length; j++) {
				points.push(tmpPoints[j]);
			}
        } else {
			console.log("Beginning non zero arc");
			var lastPoint = points[points.length-1];
			console.log(lastPoint);
			if (i === arcArr.length - 1)
            	var tmpPoints = tmp.getPointsFromArc(convertRotationstoInches(lastPoint.pos), convertNativeUnitsPer100msToInchesPerSecond(lastPoint.vel));
			else
                var tmpPoints = tmp.getPointsFromArc(convertRotationstoInches(lastPoint.pos), convertNativeUnitsPer100msToInchesPerSecond(lastPoint.vel));

            for (var j = 0; j < tmpPoints.length; j++) {
                points.push(tmpPoints[j]);
            }
        }
        console.log(points);
	}

}

function getPoint(i) {
	if(i >= waypoints.length)
		return waypoints[waypoints.length - 1];
	else
		return waypoints[i];
}

/**
 * Import from waypoint data, not json data
 */
function importData() {
	$('#upl').click();
	let u = $('#upl')[0];
	$('#upl').change(() => {
		var file =  u.files[0];
		var fr = new FileReader();
		fr.onload = function(e) {
			var c = fr.result;
			var s1 = c.split("\n");
			var tmpWaypoints = [];
			var tmpLine = [];
			let searchString1 = "new Waypoint(";
            let searchString2 = ")";
			let searchReversed1 = "public boolean isReversed() {";
			let searchReversed2 = "}";
			let searchName1 = "public class";
			let searchName2 = "implements";
			let searchAdaption1 = "PathAdapter.";
			let searchAdaption2 = "(";
            $("#title").val(c.split(searchName1)[1].split(searchName2)[0].trim());
            $("#isReversed").prop('checked', c.split(searchReversed1)[1].split(searchReversed2)[0].trim().includes("true"));

			s1.forEach((line) => {
				if (line.indexOf("//") != 0 && line.indexOf(searchString1) >= 0) {
					tmpLine.push(line);
                    tmpWaypoints.push(line.split(searchString1)[1].split(searchString2)[0].split(","));
				}
			});

			if (tmpLine[0].indexOf(searchAdaption1) >= 0) {
                var adaptStr = tmpLine[0].split(searchAdaption1)[1].split(searchAdaption2)[0].trim();
                switch (adaptStr) {
                    case "getAdaptedLeftSwitchWaypoint":
                        $("#startAdaptionValue").val("startswitchleft").prop('selected', true);
                        break;
                    case "getAdaptedRightSwitchWaypoint":
                        $("#startAdaptionValue").val("startswitchright").prop('selected', true);
                        break;
                    case "getAdaptedLeftScaleWaypoint":
                        $("#startAdaptionValue").val("startscaleleft").prop('selected', true);
                        break;
                    case "getAdaptedRightScaleWaypoint":
                        $("#startAdaptionValue").val("startscaleright").prop('selected', true);
                        break;
                    default:
                        $("#startAdaptionValue").val("startnone").prop('selected', true);
                        break;
                }
			} else {
                $("#startAdaptionValue").val("startnone").prop('selected', true);
			}

			if (tmpLine[tmpLine.length-1].indexOf(searchAdaption1) >= 0) {
				var adaptStr = tmpLine[tmpLine.length-1].split(searchAdaption1)[1].split(searchAdaption2)[0].trim();
				switch (adaptStr) {
					case "getAdaptedLeftSwitchWaypoint":
						$("#endAdaptionValue").val("endswitchleft").prop('selected', true);
						break;
					case "getAdaptedRightSwitchWaypoint":
						$("#endAdaptionValue").val("endswitchright").prop('selected', true);
						break;
					case "getAdaptedLeftScaleWaypoint":
						$("#endAdaptionValue").val("endscaleleft").prop('selected', true);
						break;
					case "getAdaptedRightScaleWaypoint":
						$("#endAdaptionValue").val("endscaleright").prop('selected', true);
						break;
					default:
						$("#endAdaptionValue").val("endnone").prop('selected', true);
						break;
				}
			} else {
                $("#endAdaptionValue").val("endnone").prop('selected', true);
			}

            waypoints = [];
            $("tbody").empty();
            tmpWaypoints.forEach((wptmp, i) => {
				var wp;
				var x = 0;
				var y = 0;
				var radius = 0;
				var speed = 0;
				var marker = "";
				if (wptmp.length >= 4) {
                    x = wptmp[0];
                    y = wptmp[1];
                    radius = wptmp[2];
                    speed = wptmp[3];
				}
				if (wptmp.length >= 5) {
                    marker = wptmp[4].replace(/"/g, "");
				}

                wp = new Waypoint(new Translation2d(x, y), speed, radius, marker);
                $("tbody").append("<tr>"
                    +"<td><input value='" + wp.position.x + "'></td>"
                    +"<td><input value='" + wp.position.y + "'></td>"
                    +"<td><input value='" + wp.radius + "'></td>"
                    +"<td><input value='" + wp.speed + "'></td>"
                    +"<td class='marker'><input placeholder='Marker' value='" + wp.marker + "'></td>"
                    +(i == 0 ? "" : "<td><button onclick='$(this).parent().parent().remove();update();''>Delete</button></td></tr>")
                );
			});
            update();

			$('input').unbind("change paste keyup");
			$('input').bind("change paste keyup", function() {
				console.log("change");
				clearTimeout(wto);
					wto = setTimeout(function() {
					update();
				}, 500);
			});

		}
		fr.readAsText(file);
	});
    update();
}

//JSON Functions
// function importData() {
// 	$('#upl').click();
// 	let u = $('#upl')[0];
// 	$('#upl').change(() => {
// 		var file =  u.files[0];
// 		var fr = new FileReader();
// 		fr.onload = function(e) {
// 			var c = fr.result;
// 			let re = /(?:\/\/\sWAYPOINT_DATA:\s)(.*)/gm;
// 			let reversed = /(?:\/\/\sIS_REVERSED:\s)(.*)/gm;
// 			let title = /(?:\/\/\sFILE_NAME:\s)(.*)/gm;
// 			//console.log();
// 			$("#title").val(title.exec(c)[1]);
// 			$("#isReversed").prop('checked', reversed.exec(c)[1].includes("true"));
// 			let jde = re.exec(c)[1];
// 			let jd = JSON.parse(jde);
// 			// console.log(jd);
// 			waypoints = []
// 			$("tbody").empty();
// 			jd.forEach((wpd) => {
// 				let wp = new Waypoint(new Translation2d(wpd.position.x, wpd.position.y), wpd.speed, wpd.radius, wpd.marker, wpd.comment);
// 				// console.log(wp);
// 				$("tbody").append("<tr>"
// 					+"<td><input value='" + wp.position.x + "'></td>"
// 					+"<td><input value='" + wp.position.y + "'></td>"
// 					+"<td><input value='" + wp.radius + "'></td>"
// 					+"<td><input value='" + wp.speed + "'></td>"
// 					+"<td class='marker'><input placeholder='Marker' value='" + wp.marker + "'></td>"
// 					+"<td class='comments'><input placeholder='Comments' value='" + wp.comment + "'></td>"
// 					+"<td><button onclick='$(this).parent().parent().remove();''>Delete</button></td></tr>"
// 				);
// 			})
// 			update();
// 			$('input').unbind("change paste keyup");
// 			$('input').bind("change paste keyup", function() {
// 				console.log("change");
// 				clearTimeout(wto);
// 					wto = setTimeout(function() {
// 					update();
// 				}, 500);
// 			});
// 		}
// 		fr.readAsText(file);
// 	});
//     update();
// }
//
// function getDataString() {
// 	var title = ($("#title").val().length > 0) ? $("#title").val() : "UntitledPath";
// 	var pathInit = "";
// 	for(var i=0; i<waypoints.length; i++) {
// 		pathInit += "        " + waypoints[i].toString() + "\n";
// 	}
// 	var startPoint = "new Translation2d(" + waypoints[0].position.x + ", " + waypoints[0].position.y + ")";
// 	var importStr = "WAYPOINT_DATA: " + JSON.stringify(waypoints);
// 	var isReversed = $("#isReversed").is(':checked');
// 	var deg = isReversed ? 180 : 0;
// 	var str = `package org.usfirst.frc.team195.robot.Autonomous.Paths;
//
// import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
// import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;
//
// import java.util.ArrayList;
//
// public class ${title} implements PathContainer {
//
//     @Override
//     public Path buildPath() {
//         ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
// ${pathInit}
//         return PathBuilder.buildPathFromWaypoints(sWaypoints);
//     }
//
//     @Override
//     public RigidTransform2d getStartPose() {
//         return new RigidTransform2d(${startPoint}, Rotation2d.fromDegrees(${deg}));
//     }
//
//     @Override
//     public boolean isReversed() {
//         return ${isReversed};
//     }
// 	// ${importStr}
// 	// IS_REVERSED: ${isReversed}
// 	// FILE_NAME: ${title}
// }`
// 	return str;
// }

function getDataString() {
	var title = ($("#title").val().length > 0) ? $("#title").val() : "UntitledPath";
	var pathInit = "";
    var startAdaptStr = $("#startAdaptionValue").val();
    var endAdaptStr = $("#endAdaptionValue").val();
	for(var i=0; i<waypoints.length; i++) {
		pathInit += "        sWaypoints.add(";

        if (i == 0) {
            switch (startAdaptStr) {
                case "startscaleleft":
                    pathInit += "PathAdapter.getAdaptedLeftScaleWaypoint(" + waypoints[i].toString() + ")";
                    break;
                case "startscaleright":
                    pathInit += "PathAdapter.getAdaptedRightScaleWaypoint(" + waypoints[i].toString() + ")";
                    break;
                case "startswitchleft":
                    pathInit += "PathAdapter.getAdaptedLeftSwitchWaypoint(" + waypoints[i].toString() + ")";
                    break;
                case "startswitchright":
                    pathInit += "PathAdapter.getAdaptedRightSwitchWaypoint(" + waypoints[i].toString() + ")";
                    break;
                default:
                    pathInit += waypoints[i].toString();
                    break;
            }
        } else if (i == waypoints.length - 1) {
            switch (endAdaptStr) {
                case "endscaleleft":
                    pathInit += "PathAdapter.getAdaptedLeftScaleWaypoint(" + waypoints[i].toString() + ")";
                    break;
                case "endscaleright":
                    pathInit += "PathAdapter.getAdaptedRightScaleWaypoint(" + waypoints[i].toString() + ")";
                    break;
                case "endswitchleft":
                    pathInit += "PathAdapter.getAdaptedLeftSwitchWaypoint(" + waypoints[i].toString() + ")";
                    break;
                case "endswitchright":
                    pathInit += "PathAdapter.getAdaptedRightSwitchWaypoint(" + waypoints[i].toString() + ")";
                    break;
                default:
                	pathInit += waypoints[i].toString();
                    break;
            }
		} else
            pathInit += waypoints[i].toString();

		pathInit += ");\n";
	}
	var startPoint = "new Translation2d(" + waypoints[0].position.x + ", " + waypoints[0].position.y + ")";
	var isReversed = $("#isReversed").is(':checked');
	var deg = isReversed ? 180 : 0;
	var str = `package org.usfirst.frc.team195.robot.Autonomous.Paths;

import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.*;
import org.usfirst.frc.team195.robot.Utilities.TrajectoryFollowingMotion.PathBuilder.Waypoint;

import java.util.ArrayList;

public class ${title} implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
${pathInit}
        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(${startPoint}, Rotation2d.fromDegrees(${deg})); 
    }

    @Override
    public boolean isReversed() {
        return ${isReversed}; 
    }
}`;
	return str;
}

function exportData() { 
	update();
	var title = ($("#title").val().length > 0) ? $("#title").val() : "UntitledPath";
	var blob = new Blob([getDataString()], {type: "text/plain;charset=utf-8"});
	saveAs(blob, title+".java");
}

function showData() {
	update();
	var title = ($("#title").val().length > 0) ? $("#title").val() : "UntitledPath";
	$("#modalTitle").html(title + ".java");
	$(".modal > pre").text(getDataString());
	showModal();
}

function showModal() {
	$(".modal, .shade").removeClass("behind");
	$(".modal, .shade").removeClass("hide");
}

function closeModal() {
	$(".modal, .shade").addClass("hide");
	setTimeout(function() {
		$(".modal, .shade").addClass("behind");
	}, 500);
}

var flipped = false;
function flipField() {
	flipped = !flipped;
	if(flipped)
		ctx.drawImage(imageFlipped, 0, 0, width, height);
	else
		ctx.drawImage(image, 0, 0, width, height);
	update();
}

function changeStartPoint() {
    if (parseInt( $($($($('tbody').children()[0]).children()[1]).children()).val() ) == startLeftY) {
        $($($($('tbody').children()[0]).children()[1]).children()).val(startRightY);
	} else if (parseInt(  $($($($('tbody').children()[0]).children()[1]).children()).val() ) == startRightY) {
        $($($($('tbody').children()[0]).children()[1]).children()).val(startLeftY);
	}
    update();
}

function canvasClick(canvas, evt) {
    var mPos = getMousePos(canvas, evt);
	addPoint(mPos.x, mPos.y);
}

function  getMousePos(canvas, evt) {
    var rect = canvas.getBoundingClientRect(); // abs. size of element

	var scaleX = width / fieldWidth / 1.5;
	var scaleY = height / fieldHeight / 1.5;
    return {
        x: (evt.clientX - rect.left) / scaleX,   // scale mouse coordinates after they have
		y: (rect.height - (evt.clientY - rect.top)) / scaleY	// been adjusted to be relative to element
    }
}

function lerpColor(color1, color2, factor) {
	var result = color1.slice();
	for (var i=0;i<3;i++) {
	result[i] = Math.round(result[i] + factor*(color2[i]-color1[i]));
	}
	return result;
}

function getColorForSpeed(speed) {
	var u = Math.max(0, Math.min(1, speed/maxSpeed));
	if(u<0.5)
		return RGBToHex(lerpColor(minSpeedColor, [255,255,0], u*2));
	return RGBToHex(lerpColor([255,255,0], maxSpeedColor, u*2-1));

}

function hexToRGB(hex) {
    var result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
    return result ? [
        parseInt(result[1], 16),
        parseInt(result[2], 16),
        parseInt(result[3], 16)
    ] : null;
}

function RGBToHex(rgb) {
    return "#" + ((1 << 24) + (rgb[0] << 16) + (rgb[1] << 8) + rgb[2]).toString(16).slice(1);
}

function getNextSpeed(prev) {
	for(var i=0; i<waypoints.length-1; i++) {
		if(waypoints[i] == prev)
			return waypoints[i+1].speed;
	}
	return 0;
}
