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
bodyDef.userData = 'box';

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

var input = new b2RayCastInput();
var output = new b2RayCastOutput();

// Global use
var p1 = new b2Vec2(10, 30);
var p2 = new b2Vec2(400, 200);
var intersectionPoint = new b2Vec2();
var intersectionNormal = new b2Vec2();
var intersectionEnd = new b2Vec2();
var normalEnd = new b2Vec2();
var reflectedEnd = new b2Vec2();

var shape = world.GetBodyList().GetFixtureList().GetShape().GetVertices();
output.fraction = 1;
console.log(world.GetBodyList().GetFixtureList());
/* console.log(world.GetBodyList().GetWorldPoint(test)); */
console.log(world.GetBodyList().GetPosition());
console.log(world.GetBodyList().GetFixtureList().GetShape().GetVertices()[0]);
console.log(bodyDef);

var i = 0;
function update() {
	i++
	world.Step(1/60 ,10 ,10);
	world.DrawDebugData();
	bodyDef.angle = i*(Math.PI/180);
	raytest();
}

function raytest() {
	input.p1 = p1;
	input.p2 = p2;
	input.maxFraction = 1;
	closestFraction = 1;

	var b = new b2BodyDef();
	var f = new b2FixtureDef();
	for(b = world.GetBodyList(); b; b = b.GetNext())    {
		/* console.log(b); */
		for(f = b.GetFixtureList(); f; f = f.GetNext()) {
			if(!f.RayCast(output, input)) {
				normalEnd = intersectionPoint;
				intersectionEnd = intersectionPoint;
				reflectedEnd = intersectionPoint;
				continue;
			}
			else if(output.fraction < closestFraction)  {
				closestFraction = output.fraction;
				/* normalEnd = output.normal; */

				// Set the one end is the vertex end of the shape
				intersectionEnd = nearest_vertex_contact(intersectionPoint, rotated_vertex_array(shape));

				// Changed function structure
				/* rotatedVector = rotated_vertex(intersectionEnd, intersectionPoint, 90, f);
				   // Expend the vector
				   rotatedVector.Multiply(100);
				   normalEnd = added_vertex(rotatedVector, intersectionPoint); */
				normalEnd = rotated_vertex(intersectionEnd, intersectionPoint, 90, f);
				// Angle of reflection
				var refl_angle = angleOfReflection(p1, intersectionPoint, normalEnd);
				console.log("Angle = " + refl_angle);
				// Changed function structure
				/* reflectedEnd_local = rotated_vertex(normalEnd, intersectionPoint, refl_angle, f);
				   reflectedEnd_local.Multiply(100);
				   reflectedEnd = added_vertex(reflectedEnd_local, intersectionPoint); */
				reflectedEnd = rotated_vertex(normalEnd, intersectionPoint, refl_angle, f);
				/* console.log(normalEnd); */
			}
		}
		/* console.log(input); */

	}
	intersectionPoint.x = p1.x + closestFraction * (p2.x - p1.x);
	intersectionPoint.y = p1.y + closestFraction * (p2.y - p1.y);
	/* normalEnd.x = normalEnd.x;
	   normalEnd.y = normalEnd.y; */
	
	context.strokeStyle = "rgb(255, 255, 255)";

	context.beginPath(); // Start the path
	context.moveTo(p1.x, p1.y); // Set the path origin
	context.lineTo(intersectionPoint.x, intersectionPoint.y); // Set the path destination
	context.closePath(); // Close the path
	context.stroke();

	context.strokeStyle = "rgb(255, 0, 0)";
	context.beginPath(); // Start the path
	context.moveTo(intersectionPoint.x, intersectionPoint.y); // Set the path origin
	context.lineTo(intersectionEnd.x, intersectionEnd.y); // Set the path destination
	context.closePath(); // Close the path
	context.stroke(); // Outline the path

	context.strokeStyle = "purple";
	context.beginPath(); // Start the path
	context.moveTo(intersectionPoint.x, intersectionPoint.y); // Set the path origin
	context.lineTo(normalEnd.x, normalEnd.y); // Set the path destination
	context.closePath(); // Close the path
	context.stroke(); // Outline the path

	context.strokeStyle = "yellow";
	context.beginPath(); // Start the path
	context.moveTo(intersectionPoint.x, intersectionPoint.y); // Set the path origin
	context.lineTo(reflectedEnd.x, reflectedEnd.y); // Set the path destination
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
			p1.x = e.layerX;
			p1.y = e.layerY;
			break;
		default:
			// Should not happen
			break;
	}
});

// Calculate the new vertex after rotation
var a = new b2Vec2(1, 2);
var b = new b2Vec2(0, 0);
var c = new b2Vec2(-1, -2);

// Return the rotated vertex of a rotation
function rotated_vertex(vertex, center_vertex, rotation, fixture){
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
	console.log(vertex);
	console.log(center_vertex);
	console.log(new_vertex);

	localVector.x = new_vertex.x * Math.cos(rotation_rad) - new_vertex.y * Math.sin(rotation_rad);
	localVector.y = new_vertex.x * Math.sin(rotation_rad) + new_vertex.y * Math.cos(rotation_rad);
	localVector.Normalize();
	testVector = added_vertex(localVector, center_vertex);
	
	if(fixture.TestPoint(testVector)){
		console.log("A");
		localVector = localVector.GetNegative();
		localVector.Multiply(50);
		vector = added_vertex(localVector, center_vertex);
	}
	else {
		localVector.Multiply(50);
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

// Return the rotated vertex in world coordinates
function rotated_vertex_array(array) {
	var rotated_array = [];
	for(i = 0; i < array.length; i++) {
		rotated_array[i] = world.GetBodyList().GetWorldPoint(array[i]);
	}
	return rotated_array;
}

/* Calculate and return the nearest point of the contact object
   contactPoint : b2Vec2
   contactObject : Array
*/
function nearest_vertex_contact(contactPoint, contactObject) {
	var distance_contact = [];
	// Store all the distance in vector form at the contact point and vertex of contact object
	for(i = 0; i < contactObject.length; i++) {
		distance_contact[i] = subtracted_vertex(contactPoint, contactObject[i]);
	}
	// Sort the lowest the highest
	distance_contact.sort(function(a, b){ return a.Length() - b.Length() });
	var test = added_vertex(distance_contact[0].GetNegative(), contactPoint);
	return test;
}

// Calculate the angle of reflection
function angleOfReflection(initial, intersection, final){
	var largerOccurance = 0;
	var u = subtracted_vertex(initial, intersection);
	var v = subtracted_vertex(final, intersection);

	var dotProduct = (u.x * v.x) + (u.y * v.y);
	console.log(u);
	console.log(v);
	console.log(dotProduct);
	var angle_cos = dotProduct / (u.Length() * v.Length());
	console.log(angle_cos);
	var angle = Math.acos(angle_cos) * 180 / Math.PI;
	if(u.x * v.y - u.y * v.x < 0)
		angle = -angle;
	return angle;
}
