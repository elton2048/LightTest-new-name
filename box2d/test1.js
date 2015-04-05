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

var debugDraw = new b2DebugDraw();
debugDraw.SetSprite ( document.getElementById ("canvas").getContext ("2d"));
/* debugDraw.SetDrawScale(30);     //define scale */
debugDraw.SetAlpha(1);
debugDraw.SetFillAlpha(.3);    //define transparency
/* debugDraw.SetLineThickness(1.0); */
debugDraw.SetFlags(b2DebugDraw.e_shapeBit | b2DebugDraw.e_jointBit);
world.SetDebugDraw(debugDraw);

window.setInterval(update,1000/60);

//box

var bodyDef = new b2BodyDef;
bodyDef.type = b2Body.b2_staticBody;
bodyDef.position.Set(200,80);

// Change TEST ANGLE
// Need conversion to degree, clockwise direction for degree
bodyDef.angle = 150 * (Math.PI/180);
bodyDef.userData = {Type:"Mirror"};

var fixDef = new b2FixtureDef;
fixDef.filter.categoryBits = 1;

fixDef.density = 10.0;
fixDef.friction = 0.5;
fixDef.restitution = .5;
/* fixDef.isSensor = true; */

fixDef.shape = new b2PolygonShape;
fixDef.shape.SetAsBox(10,50);

var box1 = world.CreateBody(bodyDef);
box1.CreateFixture(fixDef);
//end

//box

var bodyDef1 = new b2BodyDef;
bodyDef1.type = b2Body.b2_staticBody;
bodyDef1.position.Set(400,80);

// Change TEST ANGLE
// Need conversion to degree, clockwise direction for degree
bodyDef1.angle = 80 * (Math.PI/180);
bodyDef1.userData = {Type:"Mirror"};

var box2 = world.CreateBody(bodyDef1);
box2.CreateFixture(fixDef);
//end

var input = new b2RayCastInput();
var output = new b2RayCastOutput();

// Global use
var initialPoint = new b2Vec2(10, 30); // The initial point to have ray
var p2 = new b2Vec2(400, 200);
var ray_width = 15 / 2;
var ray_length;
var intersectionPoint = new b2Vec2();
var intersectionNormal = new b2Vec2();
var intersectionEnd = new b2Vec2();
var normalEnd = new b2Vec2();
var reflectedEnd = new b2Vec2();

var shape = world.GetBodyList().GetFixtureList().GetShape().GetVertices();
output.fraction = 1;
console.log(world.GetBodyList());
console.log(world.GetBodyList().GetPosition());
console.log(world.GetBodyList().GetFixtureList().GetShape().GetVertices()[0]);
console.log(bodyDef);

var i = 0;
function update() {
	i++;
	world.Step(1/60 ,10 ,10);
	world.DrawDebugData();
	raytest();
}

function raytest() {
	input.p1 = initialPoint;
	input.p2 = p2;
	ray_length = subtracted_vertex(p2, initialPoint).Length();
	input.maxFraction = 1;
	closestFraction = 1;
	var centerPoint;
	// Create two point for the representation of light beam
	var startOfBeam =
	[rotated_vertex(intersectionPoint, initialPoint, 90, "NULL", ray_width),
		rotated_vertex(intersectionPoint, initialPoint, -90, "NULL", ray_width)
	];
	var endOfBeam =
	[rotated_vertex(initialPoint, startOfBeam[0], 90, "NULL", ray_length),
		rotated_vertex(initialPoint, startOfBeam[1], -90, "NULL", ray_length),
	];
	var b = new b2BodyDef();
	var f = new b2FixtureDef();
	var rayDone = false;

	/* This part is to get the vertices of body for rendering use, not use in background calculation in this moment. */
	for(b = world.GetBodyList(); b; b = b.GetNext()) {
		/* rotated_vertex_array(b); */
	}
	/* End of get vertices*/

	// Core part to calculate the reflection
	for(b = world.GetBodyList(); b; b = b.GetNext())    {
		for(f = b.GetFixtureList(); f; f = f.GetNext()) {
			if(!f.RayCast(output, input) && !rayDone) {
				centerPoint = initialRay(intersectionPoint);
				continue;
			}
			else if(output.fraction < closestFraction)  {
				console.log(ray_length);
				if(b.GetUserData().Type == "Mirror") {
					closestFraction = output.fraction;

					// Position of intersection point at the mirror

					// Calculate the angle between intersectionPoint and initialPoint
					/* var angleOfIntersection = angleOfTwoPoints(initialPoint, initialPoint, intersectionPoint);
					   console.log("Angle of intersection: " + angleOfIntersection);
					   // Set the one end is the vertex end of the shape
					   intersectionEnd = nearest_vertex_contact(intersectionPoint, rotated_vertex_array(b));
					   //console.log("intersectionEnd:");
					   //console.log(intersectionEnd);


					   normalEnd = rotated_vertex(intersectionEnd, intersectionPoint, 90, f, 25);
					   // Angle of reflection
					   var refl_angle = angleOfTwoPoints(initialPoint, intersectionPoint, normalEnd);
					   //console.log("Angle = " + refl_angle);
					   reflectedEnd = rotated_vertex(normalEnd, intersectionPoint, refl_angle, f, 50);
					   //console.log(b.GetFixtureList().GetShape().GetVertices());
					   rayDone = true; */

					var centerPoint = rayReflection(initialPoint, intersectionPoint, b, f);
					rayDone = true;
					console.log(centerPoint);
				}
			}
		}

	}
	intersectionPoint.x = initialPoint.x + closestFraction * (p2.x - initialPoint.x);
	intersectionPoint.y = initialPoint.y + closestFraction * (p2.y - initialPoint.y);
	/* console.log(intersectionEnd); */
	/* normalEnd.x = normalEnd.x;
	   normalEnd.y = normalEnd.y; */
	
	context.strokeStyle = "rgb(255, 255, 255)";

	context.beginPath(); // Start the path
	context.moveTo(initialPoint.x, initialPoint.y); // Set the path origin
	context.lineTo(intersectionPoint.x, intersectionPoint.y); // Set the path destination
	context.closePath(); // Close the path
	context.stroke();

	context.strokeStyle = "green"

	context.beginPath(); // Start the path
	context.moveTo(startOfBeam[0].x, startOfBeam[0].y); // Set the path origin
	context.lineTo(startOfBeam[1].x, startOfBeam[1].y); // Set the path destination
	context.closePath(); // Close the path
	context.stroke();

	context.strokeStyle = "orange"
	context.beginPath(); // Start the path
	context.moveTo(startOfBeam[0].x, startOfBeam[0].y); // Set the path origin
	context.lineTo(endOfBeam[0].x, endOfBeam[0].y); // Set the path destination
	context.closePath(); // Close the path
	context.stroke();

	context.strokeStyle = "orange"
	context.beginPath(); // Start the path
	context.moveTo(startOfBeam[1].x, startOfBeam[1].y); // Set the path origin
	context.lineTo(endOfBeam[1].x, endOfBeam[1].y); // Set the path destination
	context.closePath(); // Close the path
	context.stroke();

	context.strokeStyle = "red";
	context.beginPath(); // Start the path
	context.moveTo(intersectionPoint.x, intersectionPoint.y); // Set the path origin
	context.lineTo(centerPoint.intersectionEnd.x, centerPoint.intersectionEnd.y); // Set the path destination
	context.closePath(); // Close the path
	context.stroke(); // Outline the path

	context.strokeStyle = "purple";
	context.beginPath(); // Start the path
	context.moveTo(intersectionPoint.x, intersectionPoint.y); // Set the path origin
	context.lineTo(centerPoint.normalEnd.x, centerPoint.normalEnd.y); // Set the path destination
	context.closePath(); // Close the path
	context.stroke(); // Outline the path

	context.strokeStyle = "yellow";
	context.beginPath(); // Start the path
	context.moveTo(intersectionPoint.x, intersectionPoint.y); // Set the path origin
	context.lineTo(centerPoint.reflectedEnd.x, centerPoint.reflectedEnd.y); // Set the path destination
	context.closePath(); // Close the path
	context.stroke(); // Outline the path
}

document.addEventListener("mousedown", function(e) {
	switch (e.which) {
		case 1:
			console.log(e.layerY);
			p2.x = e.layerX;
			p2.y = e.layerY;
			console.log(intersectionPoint);
			console.log(normalEnd);
			break;
		case 2:
			initialPoint.x = e.layerX;
			initialPoint.y = e.layerY;
			break;
		default:
			// Should not happen
			break;
	}
});

// Calculate the new vertex after rotation
var a = new b2Vec2(0, 2);
var b = new b2Vec2(0, 0);
var c = new b2Vec2(-1, -2);

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

function initialRay(target) {
	return {intersectionEnd: target, normalEnd: target, reflectedEnd: target};
}

function rayReflection(initial, target, body, fixture) {
	// Position of intersection point at the mirror
	
	// Calculate the angle between intersectionPoint and initialPoint
	var angleOfIntersection = angleOfTwoPoints(initial, initial, target);
	console.log("Angle of intersection: " + angleOfIntersection);
	// Set the one end is the vertex end of the shape
	var intersectionEnd = nearest_vertex_contact(target, rotated_vertex_array(body));
	//console.log(intersectionEnd);
	
	var normalEnd = rotated_vertex(intersectionEnd, target, 90, fixture, 25);
	// Angle of reflection
	var refl_angle = angleOfTwoPoints(initial, target, normalEnd);
	/* console.log("Angle = " + refl_angle); */
	var reflectedEnd = rotated_vertex(normalEnd, target, refl_angle, fixture, 50);
	return {intersectionEnd: intersectionEnd, normalEnd: normalEnd, reflectedEnd: reflectedEnd};
}
