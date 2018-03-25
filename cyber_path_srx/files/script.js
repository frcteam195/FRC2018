///////////////////////////////////////////////////////////
//Change these values to fit your robot
///////////////////////////////////////////////////////////
var robotWidth = 28.0; //inches
var robotHeight = 33.0; //inches
var wheelbaseWidth = 28; //inches
var wheelDiameter = 4.875; //inches

var startLeftY = 276;
var startCenterY = 159;
var startRightY = 48;

var fileHeader = `
package org.usfirst.frc.team195.robot.paths;
 
import org.usfirst.frc.team195.robot.util.*;
`;










///////////////////////////////////////////////////////////
//DON'T CHANGE UNLESS YOU ARE SURE
///////////////////////////////////////////////////////////
var kEncoderCountsPerRev = 4096.0;
var k100msPerMinute = 600.0;

var waypoints = [];
var points = null;
var config = [];
var path = null;
var ctx;
var width = 1656; //pixels
var height = 823; //pixels
var fieldWidth = 652; // in inches
var fieldHeight = 324; // in inches
var pointRadius = 5;
var turnRadius = 30;
var kEpsilon = 1E-9;
var image;
var imageFlipped;
var wto;


var maxSpeed = 120;
var maxSpeedColor = [0, 255, 0];
var minSpeed = 0;
var minSpeedColor = [255, 0, 0];
var pathFillColor = "rgba(150, 150, 150, 0.5)";

function changeStartPoint() {
    if (parseInt( $($($($('tbody').children()[0]).children()[1]).children()).val() ) == startLeftY) {
        $($($($('tbody').children()[0]).children()[1]).children()).val(startCenterY);
    } else if (parseInt(  $($($($('tbody').children()[0]).children()[1]).children()).val() ) == startCenterY) {
        $($($($('tbody').children()[0]).children()[1]).children()).val(startRightY);
    } else if (parseInt(  $($($($('tbody').children()[0]).children()[1]).children()).val() ) == startRightY) {
        $($($($('tbody').children()[0]).children()[1]).children()).val(startLeftY);
    }
    update();
}

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

  point() {
    ctx.fillRect(this.drawX, this.drawY, 2, 2);
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
	constructor(position, theta, comment) {
		this.position = position;
		this.theta = theta;
		this.comment = comment;
	}

	draw() {
		this.position.draw("rgba(120,120,120,0.8)");
	}

	toString() {
		var comment = (this.comment.length > 0) ? " //" + this.comment : "";
		return "sWaypoints.add(new WaypointSequence.Waypoint("+this.position.x+","+this.position.y+",Math.toRadians("+this.theta+")));" + comment;
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
		console.log(sTrans);
		console.log(eTrans);
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

	static fromPoints(a, b, c) {
		return new Arc( new Line(a, b), new Line(b, c));
	}
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
    imageFlipped = new Image();
    imageFlipped.src = 'files/field.png';
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
function create() {
	var a = new Waypoint(new Translation2d(30,30), 0,0)
	var b = new Waypoint(new Translation2d(230,30), 0,0)
	var c = new Waypoint(new Translation2d(230,230), 0,0)
	var d = new Line(a, b);
	var e = new Line(b, c);
	f = new Arc(d, e);
}

function addPoint(x, y) {
	var prev;
	var prevTheta;
	var theta;

	if(waypoints.length > 0) {
        prev = waypoints[waypoints.length - 1].position;
        prevTheta = waypoints[waypoints.length - 1].theta;
    }
	else {
        prev = new Translation2d(50, 50);
        prevTheta = 0;
    }

    if(typeof x === "undefined") {x = prev.x + 50;}

    if(typeof y === "undefined") {y = prev.y + 50;}

    x = Math.round(x);
    y = Math.round(y);
    theta = Math.round(radians2degrees(Math.atan2(y - prev.y, x - prev.x)));

	$("tbody#points").append("<tr>"
		+"<td><input value='"+(x)+"'></td>"
		+"<td><input value='"+(y)+"'></td>"
		+"<td><input value='"+theta+"'></td>"
		+"<td class='comments'><input placeholder='Comments'></td>"
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

function degrees2radians(deg) {
  return deg / 57.297469362;
}

function radians2degrees(rad) {
    return rad * 57.297469362;
}

function update() {
  clear();
	waypoints = [];
  points = new WaypointSequence($("tbody#points tr").length);
  eachPoint(function(x,y,theta,comment) {
    var pos = new Translation2d(x,y);
    waypoints.push(new Waypoint(pos, theta, comment));
		drawRotatedRect(pos, robotHeight, robotWidth, -theta, getColorForSpeed(10));
    points.addWaypoint(new WaypointSequence.Waypoint(x, y, theta));
  });
  config = new TrajectoryGenerator.Config();
  config.dt = parseFloat($("td.dt input").val());
  config.max_vel = parseFloat($("td.max_vel input").val());
  config.max_acc = parseFloat($("td.max_acc input").val());
  config.max_jerk = parseFloat($("td.max_jerk input").val());

  if (points.getNumWaypoints() > 1) {
    path = PathGenerator.makePath(points, config, wheelbaseWidth, "Curve");
    drawPath();
    $("td.time").text("" + (path.getCenterTrajectory().getNumSegments() * config.dt).toFixed(3));
  } else {
    $("td.time").text("0");
  }
  drawPoints();
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

function drawPath() {
	color = "#000";
	ctx.beginPath();
	ctx.fillStyle = color;
	ctx.strokeStyle = color;

  eachTimeSlice(function(left, right) {
    ctx.fillStyle = getColorForSpeed(left.vel);
    ctx.strokeStyle = getColorForSpeed(left.vel);
    (new Translation2d(left.x, left.y)).point();
    ctx.fillStyle = getColorForSpeed(right.vel);
    ctx.strokeStyle = getColorForSpeed(right.vel);
    (new Translation2d(right.x, right.y)).point();
  });

	ctx.fill();
	ctx.lineWidth = 0;
	ctx.stroke();
}

function eachTimeSlice(func) {
  var pair = path.getPair();
  var lpoints = pair.left.segments_;
  var cpoints = pair.center.segments_;
  var rpoints = pair.right.segments_;
  var count = cpoints.length;

  for (var i = 0; i < count; ++i) {
    func(lpoints[i], rpoints[i], i, cpoints[i]);
  }
}

function eachPoint(func) {
	$('tbody#points').children('tr').each(function () {
    var x = parseInt( $($($(this).children()).children()[0]).val() );
    if(isNaN(x)) {
      x = 0;
    }
    var y = parseInt( $($($(this).children()).children()[1]).val() );
    if(isNaN(y)) {
      y = 0;
    }
    var theta = parseFloat( $($($(this).children()).children()[2]).val() );
    if(isNaN(theta)) {
      theta = 0.0;
    }
    theta = degrees2radians(theta);
    var comment = ( $($($(this).children()).children()[3]).val() )
    func(x,y,theta,comment);
  });
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
		a.draw();
		i++;
	} while(i < waypoints.length - 2);

}

function getPoint(i) {
	if(i >= waypoints.length)
		return waypoints[waypoints.length - 1];
	else
		return waypoints[i];
}

function importData() {
	$('#upl').click();
	let u = $('#upl')[0];
	$('#upl').change(() => {
		var file =  u.files[0];
		var fr = new FileReader();
		fr.onload = function(e) {
			var c = fr.result;
			let re = /(?:\/\/\sWAYPOINT_DATA:\s)(.*)/gm;
			let reversed = /(?:\/\/\sIS_REVERSED:\s)(.*)/gm;
			let title = /(?:\/\/\sFILE_NAME:\s)(.*)/gm;
			console.log();
			$("#title").val(title.exec(c)[1]);
			$("#isReversed").prop('checked', reversed.exec(c)[1].includes("true"));
			let jde = re.exec(c)[1];
			let jd = JSON.parse(jde);
			// console.log(jd);
			waypoints = []
			$("tbody#points").empty();
			jd.forEach((wpd) => {
				let wp = new Waypoint(new Translation2d(wpd.position.x, wpd.position.y), radians2degrees(wpd.theta), wpd.comment);
				// console.log(wp);
				$("tbody#points").append("<tr>"
					+"<td><input value='" + wp.position.x + "'></td>"
					+"<td><input value='" + wp.position.y + "'></td>"
					+"<td><input value='" + radians2degrees(wpd.theta) + "'></td>"
					+"<td class='comments'><input placeholder='Comments' value='" + wp.comment + "'></td>"
					+"<td><button onclick='$(this).parent().parent().remove();update();''>Delete</button></td></tr>"
				);
			})
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
}

function getDataString() {
	var title = ($("#title").val().length > 0) ? $("#title").val() : "UntitledPath";
	var pathInit = "";
	for(var i=0; i<waypoints.length; i++) {
		pathInit += "        " + waypoints[i].toString() + "\n";
	}
	var importStr = "WAYPOINT_DATA: " + JSON.stringify(waypoints);
	var isReversed = $("#isReversed").is(':checked');
	var num_elements = path.getCenterTrajectory().getNumSegments();

	var set_segments = "double[][] centerPoints = {\n";

    var lastHeading = 0;
    var continuousHeading = 0;

	eachTimeSlice(function(left, right, i, center) {
        var nextHeading = radians2degrees(center.heading);
        if (i != 0) {
            var headingDifference = nextHeading - lastHeading;
            if (headingDifference >= 300) {
                headingDifference -= 360;
            }
            continuousHeading += headingDifference;
        }
        lastHeading = nextHeading;

		var segment = `		{${convertInchesToNativeUnits(center.pos).toFixed(3)}, ${convertIpstoTicksPer100ms(center.vel).toFixed(3)}, ${(left.dt*1000.0).toFixed(3)}, ${continuousHeading.toFixed(3)}}\n`;
		set_segments += segment;
	});

    set_segments += "\n\t};";
  
	var str = `${fileHeader}

public class ${title} extends SrxTrajectory {
	
	public ${title}() {
		super();
		this.highGear = true;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}
	
	public ${title}(boolean flipped) {
		super();
		this.highGear = true;
		this.flipped = flipped;
		centerProfile = new SrxMotionProfile(centerPoints.length, centerPoints);
	}
    
	public boolean highGear = true;

	${set_segments}
  
	// ${importStr}
	// IS_REVERSED: ${isReversed}
	// FILE_NAME: ${title}
}`
	return str;
}

function exportData() { 
	update();
	var title = ($("#title").val().length > 0) ? $("#title").val() : "UntitledPath";
	var blob = new Blob([getDataString()], {type: "text/plain;charset=utf-8"});
	saveAs(blob, title+".java", {type: "text/plain;charset=utf-8"});
}

function showData() {
	update();
	var title = ($("#title").val().length > 0) ? $("#title").val() : "UntitledPath";
	$("#modalTitle").html(title + ".java");
	$(".modal > pre").html("<code class='java'>"+getDataString()+"</code>");

	$(".modal > pre").each((i, block) => {
        hljs.highlightBlock(block);
    });

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

function lerpColor(color1, color2, factor) {
	var result = color1.slice();
	for (var i=0;i<3;i++) {
	result[i] = Math.round(result[i] + factor*(color2[i]-color1[i]));
	}
	return result;
}

function convertInchesToNativeUnits(inches) {
	return convertRotationsToNativeUnits(inches / (wheelDiameter * Math.PI));
}

function convertIpstoTicksPer100ms(ips) {
	return convertRPMtoTicksPer100ms((ips * 60.0) / (wheelDiameter * Math.PI));
}

function convertRotationsToNativeUnits(rotations) {
	return rotations * kEncoderCountsPerRev;
}

function convertRPMtoTicksPer100ms(rpm) {
	return (rpm * kEncoderCountsPerRev / k100msPerMinute);
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
