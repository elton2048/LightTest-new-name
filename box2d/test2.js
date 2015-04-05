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

var world = new b2World(new b2Vec2(0,0), true);
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
bodyDef.type = b2Body.b2_kinematicBody;
bodyDef.position.Set(30,80);
// Need conversion to degree, clockwise direction for degree
bodyDef.angle = 10*(Math.PI/180);
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

//bullet test

var bd = new b2BodyDef;
bd.type = b2Body.b2_dynamicBody;
bd.position.Set(100, 100);
/* bd.fixedRotation = true;
   bd.bullet = true;
   bd.linearDamping = 0;
   bd.gravityScale = 0;
   bd.linearVelocity = new b2Vec2(400, 200); */
var bdFd = new b2FixtureDef;
bdFd.shape = new b2CircleShape(1);
/* bdFd.shape.radius = 100.05; */
bdFd.density = 0.0;
bdFd.friction = 0.0;
bdFd.restitution = 0.99;

var box2 = world.CreateBody(bd);
box2.CreateFixture(bdFd);

//end

var input = new b2RayCastInput();
var output = new b2RayCastOutput();

// Global use
var p1 = new b2Vec2(10, 30);
var p2 = new b2Vec2(400, 200);
var intersectionPoint = new b2Vec2();

output.fraction = 1;
console.log(world.GetBodyList().GetFixtureList());

var i = 0;
function update() {
	if(isMouseDown && (!mouseJoint)) {
		var body = getBodyAtMouse();
		if(body) {
			var md = new b2MouseJointDef();
			md.bodyA = world.GetGroundBody();
			md.bodyB = body;
			md.target.Set(mouseX, mouseY);
			md.collideConnected = true;
			md.maxForce = 300.0 * body.GetMass();
			mouseJoint = world.CreateJoint(md);
			body.SetAwake(true);
		}
	}

	if(mouseJoint) {
		if(isMouseDown) {
			mouseJoint.SetTarget(new b2Vec2(mouseX, mouseY));
		} else {
			world.DestroyJoint(mouseJoint);
			mouseJoint = null;
		}
	}
	i++
		world.Step(1/60 ,10 ,10);
	world.DrawDebugData();
	bodyDef.angle = i*(Math.PI/180);
	/* raytest(); */
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
			if(!f.RayCast(output, input))
				continue;
			else if(output.fraction < closestFraction)  {
				closestFraction = output.fraction;
				intersectionNormal = output.normal;
			}
			/* console.log(f.RayCast(output, input)); */
			console.log(b);
		}
		/* console.log(input); */

	}
	intersectionPoint.x = p1.x + closestFraction * (p2.x - p1.x);
	intersectionPoint.y = p1.y + closestFraction * (p2.y - p1.y);
	context.strokeStyle = "rgb(255, 255, 255)";

	context.beginPath(); // Start the path
	context.moveTo(p1.x, p1.y); // Set the path origin
	context.lineTo(intersectionPoint.x, intersectionPoint.y); // Set the path destination
	context.closePath(); // Close the path
	context.stroke();
}

/* document.addEventListener("mousedown", function(e) {
   console.log(e.layerY);
   p2.x = e.layerX;
   p2.y = e.layerY;
   console.log(intersectionPoint);
   }); */
//mouse

var mouseX, mouseY, mousePVec, isMouseDown, selectedBody, mouseJoint;
var canvasPosition = getElementPosition(document.getElementById("canvas"));

document.addEventListener("mousedown", function(e) {
	isMouseDown = true;
	handleMouseMove(e);
	document.addEventListener("mousemove", handleMouseMove, true);
}, true);

document.addEventListener("mouseup", function() {
	document.removeEventListener("mousemove", handleMouseMove, true);
	isMouseDown = false;
	mouseX = undefined;
	mouseY = undefined;
}, true);

function handleMouseMove(e) {
	mouseX = (e.clientX - canvasPosition.x);
	mouseY = (e.clientY - canvasPosition.y);
};

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
	if(fixture.GetBody().GetType() != b2Body.b2_staticBody) {
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
