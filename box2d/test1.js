/*
   Creator: Lee Ka Chun, Elton

   Physics file for breaking the wall and reflection

   To get the reflection part. Three variables are used(centerPoint, leftPoint, rightPoint. There structure are as follow:
   startPoint(b2Vec2): The coordinates of the start point
   intersectionPoint(b2Vec2): The coordinates of the intersection point to the wall or the mirror
   The following three variable only used for reflection. The are the same as intersectionPoint if not reflection.
   intersectionEnd(b2Vec2): The coordinates of the intersection point end.
   normalEnd(b2Vec2): The coordinates of the normal end. Length of normal is fixed.
   reflectedEnd(b2Vec2): The coordinates of the reflection. End point should be depends on the length of ray. Fixed in this stage.
   
   centerPoint: The data of the center of the ray.
   leftPoint: The data of the left part of the ray.
   rightPoint: The data of the right part of the ray.
*/
var    b2Vec2 = Box2D.Common.Math.b2Vec2
    ,      b2BodyDef = Box2D.Dynamics.b2BodyDef
    ,      b2Body = Box2D.Dynamics.b2Body
    ,      b2FixtureDef = Box2D.Dynamics.b2FixtureDef
    ,      b2World = Box2D.Dynamics.b2World
    ,      b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
    ,      b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
    ,      b2ContactFilter = Box2D.Dynamics.b2ContactFilter
    ,      b2MouseJointDef =  Box2D.Dynamics.Joints.b2MouseJointDef
    ,      b2DebugDraw = Box2D.Dynamics.b2DebugDraw
    ,      b2Fixture = Box2D.Dynamics.b2Fixture
    ,      b2AABB = Box2D.Collision.b2AABB
    ,      b2WorldManifold = Box2D.Collision.b2WorldManifold
    ,      b2ManifoldPoint = Box2D.Collision.b2ManifoldPoint
    ,      b2RayCastInput = Box2D.Collision.b2RayCastInput
    ,      b2RayCastOutput = Box2D.Collision.b2RayCastOutput
    ,      b2Color = Box2D.Common.b2Color;

var world = new b2World(new b2Vec2(0,10), true);
var canvas = $('#canvas');
var context = canvas.get(0).getContext('2d');

// For building the debug drawing, delete if not used.
var debugDraw = new b2DebugDraw();
debugDraw.SetSprite ( document.getElementById ("canvas").getContext ("2d"));
debugDraw.SetAlpha(1);
debugDraw.SetFillAlpha(.3);    //define transparency
debugDraw.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit);
world.SetDebugDraw(debugDraw);
// End

window.setInterval(update,1000/60);

// Variable which can be fetched from other file.
var centerPoint;
var leftPoint;
var rightPoint;

// Variable used in the game
var wallWidth = 6;
var wallSpace = 20; // The width when the wall is broken.
var closeTime = 100; // When will the wall recover
var startWall_x = 100;
var startWall_y = 50;
var wallLength = 800;
var wallHeight = 600;
var fixedRayStrength = 800; // Ray strength for two points

var initialPoint = new b2Vec2(10, 30); // The initial point to have ray
var p2 = new b2Vec2(400, 200); // Default end point.
var ray_width = 15 / 2; // Ray width, change the first number only.


// Wall building
var startWall = new b2Vec2(startWall_x, startWall_y);
var endWall_x = new b2Vec2(startWall_x, startWall_y + wallHeight - wallWidth);
var endWall_y = new b2Vec2(startWall_x + wallLength - wallWidth, startWall_y);
var sizeWall_v = new b2Vec2(wallWidth, wallHeight);
var sizeWall_h = new b2Vec2(wallLength, wallWidth);
wallBuilding(startWall, sizeWall_v);
wallBuilding(startWall, sizeWall_h);
wallBuilding(endWall_x, sizeWall_h);
wallBuilding(endWall_y, sizeWall_v);
//end

// Obstacle setting
wallBuilding(new b2Vec2(500, 60), new b2Vec2(wallWidth, 300), 0);

/* 
   For building the mirror
   First parameter: Position
   Second parameter: Size
   Third parameter: Angle
*/
mirrorBuilding(new b2Vec2(500, 500), new b2Vec2(10, 100), 90);
//end

// Variable for Ray Cast
var input = new b2RayCastInput();
var output = new b2RayCastOutput();

// Global use
var ray_length;
var intersectionPoint = new b2Vec2();
var leftIntersectionPoint = new b2Vec2();
var rightIntersectionPoint = new b2Vec2();
var intersectionNormal = new b2Vec2();
var intersectionEnd = new b2Vec2();
var normalEnd = new b2Vec2();
var reflectedEnd = new b2Vec2();

var shape = world.GetBodyList().GetFixtureList().GetShape().GetVertices();
output.fraction = 1;
console.log(world.GetBodyList());
console.log(world.GetBodyList().GetPosition());
console.log(world.GetBodyList().GetFixtureList().GetAABB());
console.log(world.GetBodyList().GetNext().GetFixtureList());

var i = 0;

var timer = -1;

// Internal variable for calculating the wall
var wallRotation;
var wallPlace;

function update() {
	i++;
	world.Step(1/60 ,10 ,10);
	world.DrawDebugData();
	raytest();

	// Part to break the wall and recover
	if(isMouseDown){
		var body = getBodyAtMouse();
		if(body) {
			console.log(body.GetFixtureList());
			var aabb = body.GetFixtureList().GetAABB();
			var upperLength;
			var upperSize;
			var lowerStartWall;
			var lowerStart_y;
			if(body.GetUserData().Rotation == "Horizontal") {
				// Left part of wall
				if((mouseX - aabb.lowerBound.x) / 2 > wallSpace){
					upperLength = mouseX - aabb.lowerBound.x;
				} else {
					upperLength = wallSpace / 2;
				}
				upperSize = new b2Vec2(upperLength - wallSpace / 2, wallWidth);
				wallBuilding(body.GetUserData().Start, upperSize);

				// Right part of wall
				lowerLength = aabb.upperBound.x - mouseX;
				if(lowerLength < wallSpace) {
					lowerSize = new b2Vec2(0, wallWidth);
					lowerStartWall = new b2Vec2(aabb.upperBound.x, body.GetUserData().Start.y);
				} else {
					lowerSize = new b2Vec2(lowerLength - wallSpace, wallWidth);
					lowerStartWall = new b2Vec2(mouseX + wallSpace, body.GetUserData().Start.y);
				}
				wallBuilding(lowerStartWall, lowerSize);
			}
			// Vertical part
			else if(body.GetUserData().Rotation == "Vertical") {
				// Upper part of wall
				if((mouseY - aabb.lowerBound.y) / 2 > wallSpace){
					upperLength = mouseY - aabb.lowerBound.y;
				} else {
					upperLength = wallSpace / 2;
				}
				upperSize = new b2Vec2(wallWidth, upperLength - wallSpace / 2);
				wallBuilding(body.GetUserData().Start, upperSize);

				// Lower part of wall
				lowerLength = aabb.upperBound.y - mouseY;
				lowerSize = new b2Vec2(wallWidth, lowerLength - wallSpace);
				lowerStartWall = new b2Vec2(body.GetUserData().Start.x, mouseY + wallSpace);

				if(lowerLength < wallSpace) {
					lowerSize = new b2Vec2(wallWidth, 0);
					lowerStartWall = new b2Vec2(body.GetUserData().Start.x, aabb.upperBound.y);
				} else {
					lowerSize = new b2Vec2(wallWidth, lowerLength - wallSpace);
					lowerStartWall = new b2Vec2(body.GetUserData().Start.x, mouseY + wallSpace);
				}
				wallBuilding(lowerStartWall, lowerSize);
			}
			// Record the wall rotation
			wallRotation = body.GetUserData().Rotation;
			wallPlace = body.GetUserData().Start;
			
			world.DestroyBody(body);
			timer = 1;
			isMouseDown = false;
		}
	}
	if(timer > 0 && timer < closeTime) {
		timer++;
	}
	if(timer == closeTime) {
		if(wallRotation == "Horizontal")
			wallBuilding(wallPlace, sizeWall_h);
		else if(wallRotation == "Vertical")
			wallBuilding(wallPlace, sizeWall_v);
			
		world.DestroyBody(world.GetBodyList().GetNext());
		world.DestroyBody(world.GetBodyList().GetNext());
		timer = -1;
	}
}

function raytest() {
	// Ray length based on two points centerPoint
	ray_length = subtracted_vertex(p2, initialPoint).Length();
	
	input.maxFraction = 1;
	closestFraction = 1;
	// Create two point for the representation of light beam
	var startOfBeam =
	[rotated_vertex(intersectionPoint, initialPoint, -90, "NULL", ray_width),
		rotated_vertex(intersectionPoint, initialPoint, 90, "NULL", ray_width)
	];
	var endOfBeam =
	[rotated_vertex(initialPoint, startOfBeam[0], -90, "NULL", ray_length),
		rotated_vertex(initialPoint, startOfBeam[1], 90, "NULL", ray_length),
	];
	var b = new b2BodyDef();
	var f = new b2FixtureDef();
	var rayDone = false;

	// Part to get the fraction point for three rays
	var fractionPoint = [];
	for(i = 0; i < 3; i++) {
		switch(i){
			case 0:
				input.p1 = initialPoint;
				input.p2 = p2;
				break;
			case 1:
				input.p1 = startOfBeam[0];
				input.p2 = endOfBeam[0];
				break;
			case 2:
				input.p1 = startOfBeam[1];
				input.p2 = endOfBeam[1];
				break;
		}
		rayDone = false;
		for(b = world.GetBodyList(); b; b = b.GetNext()) {
			for(f = b.GetFixtureList(); f; f = f.GetNext()) {
				if(!f.RayCast(output, input) && !rayDone) {
					fractionPoint[i] = 1;
					continue;
				}
				else if(output.fraction < closestFraction) {
					fractionPoint[i] = output.fraction;
					rayDone = true;
					/* console.log(fractionPoint); */
				}
			}
		}
	}
	/* End of get vertices*/

	// Get the point after reflection
	centerPoint = rayReflection_v2(initialPoint, p2);
	leftPoint = rayReflection_v2(startOfBeam[0], endOfBeam[0]);
	rightPoint = rayReflection_v2(startOfBeam[1], endOfBeam[1]);
	firstReflected = rayReflection_v2(centerPoint.intersectionPoint, centerPoint.reflectedEnd);

	intersectionPoint.x = initialPoint.x + closestFraction * (p2.x - initialPoint.x);
	intersectionPoint.y = initialPoint.y + closestFraction * (p2.y - initialPoint.y);

	// For drawing the debug information in debug plant, delete if not used
	debugDrawLine("rgb(255, 255, 255)", centerPoint.startPoint, centerPoint.intersectionPoint);
	debugDrawLine("orange", leftPoint.startPoint, leftPoint.intersectionPoint);
	debugDrawLine("green", leftPoint.startPoint, rightPoint.startPoint);
	debugDrawLine("orange", rightPoint.startPoint, rightPoint.intersectionPoint);
	debugDrawLine("red", centerPoint.intersectionPoint, centerPoint.intersectionEnd);
	debugDrawLine("red", leftPoint.intersectionPoint, leftPoint.intersectionEnd);
	debugDrawLine("purple", centerPoint.intersectionPoint, centerPoint.normalEnd);
	debugDrawLine("yellow", firstReflected.startPoint, firstReflected.reflectedEnd);
	// End

}

// Mouse part
var mouseX, mouseY, mousePVec, isMouseDown, selectedBody, mouseJoint;
var isMouseUp;
var canvasPosition = getElementPosition(document.getElementById("canvas"));
function handleMouseMove(e) {
	mouseX = (e.clientX - canvasPosition.x);
	mouseY = (e.clientY - canvasPosition.y);
}
function getBodyAtMouse() {
	mousePVec = new b2Vec2(mouseX, mouseY);
	var aabb = new b2AABB();
	aabb.lowerBound.Set(mouseX - 0.001, mouseY - 0.001);
	aabb.upperBound.Set(mouseX + 0.001, mouseY + 0.001);
	
	// Query the world for overlapping shapes.
	
	selectedBody = null;
	world.QueryAABB(getBodyCB, aabb);
	return selectedBody;
}

function getBodyCB(fixture) {
	if(fixture.GetBody().GetType() == b2Body.b2_staticBody) {
		if(fixture.GetShape().TestPoint(fixture.GetBody().GetTransform(), mousePVec)) {
		    selectedBody = fixture.GetBody();
		    return false;
		}
	}
	return true;
}

function getElementPosition(element) {
	var elem=element, tagname="", x=0, y=0;
	
	while((typeof(elem) == "object") && (typeof(elem.tagName) != "undefined")) {
		y += elem.offsetTop;
		x += elem.offsetLeft;
		tagname = elem.tagName.toUpperCase();
		
		if(tagname == "BODY")
		    elem=0;
		
		if(typeof(elem) == "object") {
		    if(typeof(elem.offsetParent) == "object")
		        elem = elem.offsetParent;
		}
	}
	
	return {x: x, y: y};
}

document.addEventListener("click", function(e) {
	isMouseUp = false;
	switch (e.which) {
		// Left click
		case 1:
			// This version is to have fixed ray strength. Mouse point only affect the direction of the light
			var tempPoint = new b2Vec2(e.layerX - initialPoint.x, e.layerY - initialPoint.y);
			tempPoint.Normalize();
			tempPoint.Multiply(fixedRayStrength);
			console.log(tempPoint);
			p2 = added_vertex(tempPoint, initialPoint);
			/* p2.x = e.layerX;
			   p2.y = e.layerY; */
			break;
		// Middle click
		case 2:
			initialPoint.x = e.layerX;
			initialPoint.y = e.layerY;
			break;
		// Right click
		case 3:
			if(timer == -1) {
				isMouseDown = true;
				handleMouseMove(e);
			}
			break;
		default:
			// Should not happen
			break;
	}
});
// Right click has no menu
$(document).bind("contextmenu", function(e) {
    return false;
});

function debugDrawLine(colour, start, end) {
	context.strokeStyle = colour;

	context.beginPath();
	context.moveTo(start.x, start.y);
	context.lineTo(end.x, end.y);
	context.closePath();
	context.stroke();
}

/*
   START PART FOR THE FUNCTION IN INTERNAL CALCULATION
   CHANGE MUST BE MADE CAUTIOUSLY
*/

/* Calculate the rotated vertex of a rotation
   Perform the action like a vector rotation
   vertex : b2Vec2(), head of vertex
   center_vertex : b2Vec2(), tail of vertex
   rotation : float, in degree representation
   fixture : b2Fixture, decide whether a fixture is existed for getting negative vector.
   strength : float, decide the strength of the vector.
   
   Return : The rotated vector head position as vertex with strength.
*/
function rotated_vertex(vertex, center_vertex, rotation, fixture, strength){
	// Convert degree to radian
	var rotation_rad = rotation * (Math.PI / 180);
	// Produce the original vector which from the given vertices
	var new_vertex = new b2Vec2();
	/* console.log(new_vertex); */
	new_vertex.SetV(vertex);
	new_vertex.Subtract(center_vertex);
	var localVector = new b2Vec2();
	var testVector = new b2Vec2();
	var vector = new b2Vec2();
	/* console.log(vertex);
	   console.log(center_vertex);
	   console.log(new_vertex); */

	localVector.x = new_vertex.x * Math.cos(rotation_rad) - new_vertex.y * Math.sin(rotation_rad);
	localVector.y = new_vertex.x * Math.sin(rotation_rad) + new_vertex.y * Math.cos(rotation_rad);
	localVector.Normalize();
	testVector = added_vertex(localVector, center_vertex);
	
	if(fixture != "NULL") {
		if(fixture.TestPoint(testVector)){
			localVector = localVector.GetNegative();
			localVector.Multiply(strength);
			vector = added_vertex(localVector, center_vertex);
		} else {
			localVector.Multiply(strength);
			vector = added_vertex(localVector, center_vertex);
		}
	} else {
		localVector.Multiply(strength);
		vector = added_vertex(localVector, center_vertex);
	}
	return vector;
}

// Perform an addition of two vertex without affect the existing vertex
function added_vertex(vertex, center_vertex){
	// Produce the original vector which from the given vertices
	var _vertex = new b2Vec2();
	_vertex.x = vertex.x;
	_vertex.y = vertex.y;
	_vertex.Add(center_vertex);
	var vector = _vertex;

	return vector;
}

// Perform a subtraction of two vertex without affect the existing vertex
function subtracted_vertex(vertex, center_vertex){
	// Produce the original vector which from the given vertices
	var _vertex = new b2Vec2();
	_vertex.x = vertex.x;
	_vertex.y = vertex.y;
	_vertex.Subtract(center_vertex);
	var vector = _vertex;

	return vector;
}

/* Return the rotated vertex of the object in world coordinates

*/
function rotated_vertex_array(bodyObject) {
	var rotated_array = [];
	var localVertices = bodyObject.GetFixtureList().GetShape().GetVertices();
	for(i = 0; i < localVertices.length; i++) {
		rotated_array[i] = bodyObject.GetWorldPoint(localVertices[i]);
	}
	/* console.log("rotated_array: ");
	   console.log(rotated_array); */
	return rotated_array;
}

/* Calculate and return the nearest point of the contact object
   contactPoint : b2Vec2
   contactObject : Array
*/
function nearest_vertex_contact(contactPoint, contactObject) {
	var distance_contact = [];
	/* console.log("contactObject: ");
	   console.log(contactObject); */
	// Store all the distance in vector form at the contact point and vertex of contact object
	for(i = 0; i < contactObject.length; i++) {
		distance_contact[i] = subtracted_vertex(contactPoint, contactObject[i]);
	}
	// Sort the lowest the highest
	distance_contact.sort(function(a, b){ return a.Length() - b.Length() });
	var test = added_vertex(distance_contact[0].GetNegative(), contactPoint);
	/* console.log("nearest_vertex_contact returns:");
	   console.log(test); */
	return test;
}

/* Calculate the angle of reflection
   If the initial and the intersection are the same, calculate the angle against x-axis
   initial : The initial point
   intersection : Point that link between initial and final
   final : The final point

   Return : Angle in degree
*/
function angleOfTwoPoints(initial, intersection, final){
	var largerOccurance = 0;
	var u = subtracted_vertex(initial, intersection);
	var v = subtracted_vertex(final, intersection);
	/* console.log(subtracted_vertex(intersection, initial).Length()); */
	if(subtracted_vertex(intersection, initial).Length() == 0){
		initial = new b2Vec2(intersection.x + 1, intersection.y);
		u = subtracted_vertex(initial, intersection);
		v = subtracted_vertex(final, intersection);
	}

	var dotProduct = (u.x * v.x) + (u.y * v.y);
	var angle_cos = dotProduct / (u.Length() * v.Length());
	var angle = Math.acos(angle_cos) * 180 / Math.PI;
	if(u.x * v.y - u.y * v.x < 0)
		angle = -angle;
	return angle;
}

/* To initialize the ray information

   Return : Object
*/
function initialRay(initial, fraction, target) {
	var intersectionPoint = new b2Vec2();
	intersectionPoint.x = initial.x + fraction * (target.x - initial.x);
	intersectionPoint.y = initial.y + fraction * (target.y - initial.y);
	return {startPoint: initial,
		intersectionPoint: intersectionPoint,
		intersectionEnd: intersectionPoint,
		normalEnd: intersectionPoint,
		reflectedEnd: intersectionPoint};
}

function rayReflection_v2(initial, final) {
	input.p1 = initial;
	input.p2 = final;
	output.fraction = 1;
	var rayDone = false;
	var rayReflect = false;
	var fraction = 1;
	var intersectionPoint = new b2Vec2();
	var angleOfIntersection;
	var intersectionEnd = new b2Vec2();
	var normalEnd = new b2Vec2();
	var reflectionAngle;
	var reflectedEnd = new b2Vec2();

	var targetBody;
	var targetFixture;
	
	for(body = world.GetBodyList(); body; body = body.GetNext())    {
		for(fixture = body.GetFixtureList(); fixture; fixture = fixture.GetNext()) {
			if(!fixture.RayCast(output, input) && !rayDone) {
				/* console.log("NOT"); */
				fraction = 1;
				continue;
			}
			else if(output.fraction < closestFraction)  {
				/* console.log("hit"); */
				if(fraction > output.fraction){ 
					fraction = output.fraction;
					switch(body.GetUserData().Type) {
						// Position of intersection point at the mirror
						case "Mirror":
							rayReflect = true;
							targetBody = body;
							targetFixture = fixture;
							break;
						case "Wall":
							rayReflect = false;
							break;
					}
				}
				rayDone = true;

			}
		}
	}
	intersectionPoint.x = initial.x + fraction * (final.x - initial.x);
	intersectionPoint.y = initial.y + fraction * (final.y - initial.y);
	if(rayReflect) {
		// Calculate the angle between intersectionPoint and initialPoint
		angleOfIntersection = angleOfTwoPoints(initial, initial, intersectionPoint);
		/* console.log("Angle of intersection: " + angleOfIntersection); */
		// Set the one end is the vertex end of the shape
		intersectionEnd = nearest_vertex_contact(intersectionPoint, rotated_vertex_array(targetBody));
		//console.log(intersectionEnd);
		
		normalEnd = rotated_vertex(intersectionEnd, intersectionPoint, 90, targetFixture, 25);
		// Angle of reflection
		reflectionAngle = angleOfTwoPoints(initial, intersectionPoint, normalEnd);
		/* console.log("Angle = " + refl_angle); */
		reflectedEnd = rotated_vertex(normalEnd, intersectionPoint, reflectionAngle, targetFixture, ray_length);
	} else {
		intersectionEnd = intersectionPoint;
		normalEnd = intersectionPoint;
		reflectedEnd = intersectionPoint;
	}
	/* console.log(intersectionPoint);
	   console.log(intersectionEnd); */
	return {startPoint: initial,
		intersectionPoint: intersectionPoint,
		intersectionEnd: intersectionEnd,
		normalEnd: normalEnd,
		reflectedEnd: reflectedEnd};
}

function wallBuilding(startPoint, size) {
	var position = new b2Vec2();
	position.x = startPoint.x + size.x / 2;
	position.y = startPoint.y + size.y / 2;
	// Wall building
	
	var wall1 = new b2BodyDef;
	wall1.type = b2Body.b2_staticBody;
	wall1.position.Set(position.x, position.y);
	if(size.x == wallWidth) 
		wall1.userData = {Type: "Wall", Start: startPoint, Rotation: "Vertical"};
	else
		wall1.userData = {Type: "Wall", Start: startPoint, Rotation: "Horizontal"};
	
	var wall1Fix = new b2FixtureDef;
	wall1Fix.filter.categoryBits = 1;
	
	wall1Fix.density = 10.0;
	wall1Fix.friction = 0.5;
	wall1Fix.restitution = .5;
	
	wall1Fix.shape = new b2PolygonShape;
	wall1Fix.shape.SetAsBox(size.x / 2, size.y / 2);
	
	var wall1World = world.CreateBody(wall1);
	wall1World.CreateFixture(wall1Fix);
	//end

}

function mirrorBuilding(startPoint, size, angle) {
	var position = new b2Vec2();
	position.x = startPoint.x + size.x;
	position.y = startPoint.y + size.y;
	
	var mirror = new b2BodyDef;
	mirror.type = b2Body.b2_staticBody;
	mirror.position.Set(position.x, position.y);
	mirror.angle = angle * (Math.PI/180);
	mirror.userData = {Type: "Mirror"};

	var mirrorFix = new b2FixtureDef;
	mirrorFix.shape = new b2PolygonShape;
	mirrorFix.shape.SetAsBox(size.x, size.y);
	
	var mirrorWorld = world.CreateBody(mirror);
	mirrorWorld.CreateFixture(mirrorFix);
}
