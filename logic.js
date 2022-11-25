canvcreate("", 500, 500);
canv.width = window.innerWidth;
canv.height = window.innerHeight;
window.onresize = function () {
	canv.width = window.innerWidth;
	canv.height = window.innerHeight;
}

var dt = 0.2;

var frame = 0;
var startTime = new Date();
var deltaTime = 0;
var fps = 0;
var rFps = 0;

var Recording = new Record();

var showJoints = false;

var debugGraph = new NewGraph([ 10, canv.height - 400-10], 400, 400, 500, 30);
debugGraph.data[0] = {name:"Body velocity",color:"red",data:[]};
debugGraph.data[1] = {name:"Y velocity",color:"rgba(0,255,0,0.3)",data:[]};

var needDrawDebugGraph = true;

class Particle {
	constructor(Position = { x: 0, y: 0 }, Velocity = { x: 0, y: 0 }, Mass = 1, AttractRadius = 1, Damping = 1) {
		this.startPosition = { x: Position.x, y: Position.y };

		this.position = Position;
		this.velocity = Velocity;
		this.mass = Mass;
		this.attractRadius = AttractRadius;
		this.damping = Damping;

		this.isFreezed = false;

		this.deformingIntensity = 0.01;

		this.groupIndex = 0;
		this.selfId = -1;
		this.groupLink;
		this.inGroupID = 0

		this.startDeltas = [];
		this.joints = [];
		// this.recalculateDistances();

		//DEBUG VARS
		this.appearFrame = 0

	}
	findJoint(particle) {
		for (let i = 0; i < this.joints.length; i++) {
			if (this.joints[i] == particle) return i;
		}
		return -1;
	}
	recalculateDistances(position = this.position, saveDistances = false) {
		let jointsToDelete = [];
		for (let i = 0; i < this.joints.length; i++) {
			if (this.startDeltas[i] != undefined && saveDistances) {
				const preDelta = this.startDeltas[i];
				const startDist = math.magnitude(this.startDeltas[i]);
				this.startDeltas[i] = { x: this.joints[i].position.x - position.x, y: this.joints[i].position.y - position.y };

				if (Math.abs(math.magnitude(this.startDeltas[i]) - startDist) > 200) {
					// jointsToDelete.push(i);
					continue;
				}

				this.startDeltas[i] = math.normalize(this.startDeltas[i]);
				this.startDeltas[i].x *= startDist;
				this.startDeltas[i].y *= startDist;
			} else {
				this.startDeltas[i] = { x: this.joints[i].position.x - position.x, y: this.joints[i].position.y - position.y };
			}
		}
		for (let i = 0; i < jointsToDelete.length; i++) {
			// console.log(this.joints[jointsToDelete[i]]);
			if (this.joints[jointsToDelete[i]] == undefined) continue;
			let otherParticleJoint = this.joints[jointsToDelete[i]].findJoint(this);
			this.joints[jointsToDelete[i]].joints.splice(otherParticleJoint, 1);
			this.joints[jointsToDelete[i]].startDeltas.splice(otherParticleJoint, 1);
			this.joints.splice(jointsToDelete[i], 1);
			this.startDeltas.splice(jointsToDelete[i], 1);
		}
	}
	deformStructure() {
		if (this.joints.length > 0) {
			let cent = pGroups[this.groupIndex].GetCentere();

			let delta = {
				x: (-this.startDeltas[0].x + this.joints[0].position.x) - this.position.x,
				y: (-this.startDeltas[0].y + this.joints[0].position.y) - this.position.y
			};

			let coff = Math.sqrt(delta.x * delta.x + delta.y * delta.y);
			coff = Math.pow(coff, 2);
			coff *= 0.1;
			let clr = inRgb(coff, 0, 255 - coff);
			// d.line(-(this.startDeltas[0].x-this.joints[0].position.x),-(this.startDeltas[0].y-this.joints[0].position.y),this.position.x,this.position.y,clr);
		}
	}
	disAttractTo(ePartic) {
		let delta = math.distance([ePartic.position.x, ePartic.position.y], [this.position.x, this.position.y]);
		// let force = (ePartic.mass / (this.attractRadius*Math.pow(delta[0],1)))-(ePartic.mass / Math.pow(delta[0],2));
		let force = -this.attractRadius / Math.pow(delta[0], 1);
		this.velocity.x += (delta[1] / delta[0]) * force * dt;
		this.velocity.y += (delta[2] / delta[0]) * force * dt;
	}
	springByDistances() {
		let springForceDirection = { x: 0, y: 0 };
		let forces = 0;

		let groupCentre = pGroups[this.groupIndex].GetCentere();
		let jointsAverageVelocity = { x: this.velocity.x, y: this.velocity.y };
		if (this.joints.length > 0) {
			for (let i = 0; i < this.joints.length; i++) {
				jointsAverageVelocity.x += this.joints[i].velocity.x;
				jointsAverageVelocity.y += this.joints[i].velocity.y;
			}
			jointsAverageVelocity.x /= this.joints.length;
			jointsAverageVelocity.y /= this.joints.length;
		}

		// let groupAverageVelocity = pGroups[this.groupIndex].averageVelocity;
		for (let i = 0; i < this.joints.length; i++) {
			if (this.joints[i].velocity.x == 0 && this.joints[i].velocity.y == 0) continue;
			let x = this.startDeltas[i];
			let c = this.joints[i];

			this.spring(c.position, math.magnitude(x));// {x:this.position.x+defLocalPosition.x,y:this.position.y+defLocalPosition.y}
			if (showJoints)
				d.line(this.position.x, this.position.y, c.position.x, c.position.y, "red", 0.1);

			// this.joints[i].velocity.x += (newJointVel.x-this.joints[i].velocity.x)*this.damping;
			// this.joints[i].velocity.y += (newJointVel.y-this.joints[i].velocity.y)*this.damping;
		}

		// let dot1 = math.dot(this.velocity,groupAverageVelocity);
		// this.velocity.x = (1-this.damping)*this.velocity.x + this.damping*(groupAverageVelocity.x*dot1);
		// this.velocity.y = (1-this.damping)*this.velocity.y + this.damping*(groupAverageVelocity.y*dot1);
		this.velocity.x += (jointsAverageVelocity.x - this.velocity.x) * this.damping * dt;
		this.velocity.y += (jointsAverageVelocity.y - this.velocity.y) * this.damping * dt;
		// d.ray(this.position.x,this.position.y,this.velocity,math.magnitude(this.velocity)*3,"red");
	}
	spring(target, defaultDistance) {
		let distance = math.magnitude({ x: this.position.x - target.x, y: this.position.y - target.y });
		let direction = { x: (this.position.x - target.x) / distance, y: (this.position.y - target.y) / distance };

		let force = this.mass * (distance - defaultDistance);
		this.velocity.x += direction.x * -force * dt;
		this.velocity.y += direction.y * -force * dt;
	}
	reactOnWall(wall, error = 0) {
		let wallNormal = wall.getNormal();
		let d = math.dot(wallNormal, this.velocity);
		this.velocity = { x: this.velocity.x - 2 * d * wallNormal.x * 0.85, y: this.velocity.y - 2 * d * wallNormal.y * 0.85 };

		this.position.x += wallNormal.x * (error - wall.width);
		this.position.y += wallNormal.y * (error - wall.width);
	}
	reactOnDynamicWall(wall, error = 0) {
		const restitution = 0.9;

		let wallNormal = wall.getNormal();
		let d = math.dot(wallNormal, this.velocity);
		this.velocity = { x: (this.velocity.x - 2 * d * wallNormal.x) * restitution, y: (this.velocity.y - 2 * d * wallNormal.y) * restitution };

		// TODO:!!transit other group velocity by there normal, not by current!!
		// wall.sourceParticle1.velocity = {x:(wall.sourceParticle1.velocity.x-2*d*-wallNormal.x)*restitution/2.5,y:(wall.sourceParticle1.velocity.y-2*d*-wallNormal.y)*restitution/2.5};
		// wall.sourceParticle2.velocity = {x:(wall.sourceParticle2.velocity.x-2*d*-wallNormal.x)*restitution/2.5,y:(wall.sourceParticle2.velocity.y-2*d*-wallNormal.y)*restitution/2.5};

		this.position.x += wallNormal.x * (error - wall.width) / 3;
		this.position.y += wallNormal.y * (error - wall.width) / 3;

		wall.sourceParticle1.position.x += -wallNormal.x * (error - wall.width) / 3;
		wall.sourceParticle1.position.y += -wallNormal.y * (error - wall.width) / 3;
		wall.sourceParticle2.position.x += -wallNormal.x * (error - wall.width) / 3;
		wall.sourceParticle2.position.y += -wallNormal.y * (error - wall.width) / 3;
	}
	isIntersectWithWall(wall) {
		let wallNormal = wall.getNormal();
		let vecToPoint = { x: wall.point1.x - this.position.x, y: wall.point1.y - this.position.y };

		let err = math.dot(wallNormal, vecToPoint);
		let tangentErr = math.dot(math.normalize({ x: wall.point2.x - wall.point1.x, y: wall.point2.y - wall.point1.y }), vecToPoint);

		if (Math.abs(err) < wall.width && Math.abs(tangentErr) > 0 && Math.abs(tangentErr) < wall.getLength()) {
			return { isInter: true, error: err };
		}
		return { isInter: false, error: err };
	}
	IsIntersectWithGroup(group) {
		for (let pInG = 0; pInG < group.particles.length; pInG++) {
			var wall = group.GetAsWall(pInG);
			let data = this.isIntersectWithWall(wall);
			// wall.draw();
			if (data.isInter)
				return { isInter: true, error: data.error, collIndex: pInG, Wall: wall };
		}

		return { isInter: false, error: 0, collIndex: 0, Wall: undefined };
	}
}
class Wall {
	constructor(Point1, Point2, Width = 8) {
		this.point1 = Point1;
		this.point2 = Point2;
		this.width = Width;

		this.sourceParticle1 = null;
		this.sourceParticle2 = null;

		this.isNormalInverted = false;
	}
	draw() {
		d.line(this.point1.x, this.point1.y, this.point2.x, this.point2.y, "red", 1.5);
		d.ray((this.point1.x + this.point2.x) / 2, (this.point1.y + this.point2.y) / 2, this.getNormal(), 10, "green");

		// d.circle(this.point2.x,this.point2.y,2,"red");
		// d.circle(this.point2.x,this.point2.y,2,"red");
	}
	getNormal() {
		let delta = { x: this.point2.x - this.point1.x, y: this.point2.y - this.point1.y };
		delta = math.normalize(delta);
		if (this.isNormalInverted)
			return { x: delta.y, y: -delta.x };
		return { x: -delta.y, y: delta.x };
	}
	fixNormalToCircle(centere) {
		let wallDirection = {
			x: this.point2.x - this.point1.x,
			y: this.point2.y - this.point1.y
		};
		let toCentereDirection = {
			x: centere.x - this.point1.x,
			y: centere.y - this.point1.y
		};
		let direction = math.pseudoDot(toCentereDirection, wallDirection);
		if (direction > 0)
			this.isNormalInverted = true;
		else
			this.isNormalInverted = false;
	}
	getLength() {
		return math.magnitude({ x: this.point2.x - this.point1.x, y: this.point2.y - this.point1.y });
	}
}
class ParticlesGroup {
	constructor(Particles = []) {
		this.particles = Particles;
		this.selfId = 0;

		this.averageVelocity = this.GetAverageVelocity();

		let centere = this.GetCentere();
		this.particlesStartLocalPosition = [];
		for (let i = 0; i < this.particles.length; i++) {
			this.particlesStartLocalPosition.push({
				x: this.particles[i].position.x - centere.x,
				y: this.particles[i].position.y - centere.y
			});
		}
	}
	RecreateJoints(Quality =1) {
		for (let i = 0; i < this.particles.length; i++) {
			for (let j = 0; j < this.particles.length; j+=Quality) {
				if (i == j) continue;
				this.particles[i].joints.push(this.particles[j]);
			}
			this.particles[i].recalculateDistances();
		}
	}
	Draw() {
		ctx.strokeStyle = "white";
		ctx.beginPath();
		for (let i = 0; i < this.particles.length; i++) {
			let nextI = i + 1;
			if (nextI > this.particles.length - 1)
				nextI = 0;
			if (this.particles[i].joints.length == 0 || this.particles[nextI].joints.length == 0) {
				d.rect(this.particles[i].position.x, this.particles[i].position.y, 2, 2, "white", "white");
				continue;
			}
			// ctx.moveTo(this.particles[i].position.x,this.particles[i].position.y);
			// ctx.lineTo(this.particles[nextI].position.x,this.particles[nextI].position.y);
			// Recording.AddData({ id: this.selfId, position: { x: this.particles[i].position.x, y: this.particles[i].position.y } });
			// d.txt(i,this.particles[i].position.x,this.particles[i].position.y);
			d.rect(this.particles[i].position.x, this.particles[i].position.y, 2, 2, "white", "white");
		}
		ctx.stroke();
		// for (let i = 0; i < this.particles.length; i++) {
		// 	let w = this.GetAsWall(i);
		// 	d.ray((w.point1.x+w.point2.x)/2,(w.point1.y+w.point2.y)/2,w.getNormal(),6,"green");
		// }
	}
	GetCentere() {
		let pos = { x: 0, y: 0 };
		for (let i = 0; i < this.particles.length; i++) {
			// if (isNaN(this.particles[i].position)) console.log(i);
			pos.x += this.particles[i].position.x;
			pos.y += this.particles[i].position.y;
		}
		return { x: pos.x / this.particles.length, y: pos.y / this.particles.length };
	}
	GetAverageVelocity() {
		let averVelocity = { x: 0, y: 0 };
		for (let j = 0; j < this.particles.length; j++) {
			// if (this.joints[j].velocity.x == 0 && this.joints[j].velocity.y == 0) continue;

			averVelocity.x += this.particles[j].velocity.x;
			averVelocity.y += this.particles[j].velocity.y;
		}
		averVelocity.x /= this.particles.length;
		averVelocity.y /= this.particles.length;
		return averVelocity;
	}
	GetAsWall(i) {
		let nextI = i + 1;
		if (nextI > this.particles.length - 1) nextI = 0;
		let wall = new Wall(this.particles[i].position, this.particles[nextI].position, 4);
		wall.sourceParticle1 = this.particles[i];
		wall.sourceParticle2 = this.particles[nextI];
		wall.fixNormalToCircle(this.GetCentere());
		return wall;
	}
	GetAverageDeformationLevel() {
		let jointsCount = 0;
		let deform = 0;
		for (let i = 0; i < this.particles.length; i++) {
			for (let j = 0; j < this.particles[i].joints.length; j++) {
				let p1 = this.particles[i].position;
				let p2 = this.particles[i].joints[j].position;
				deform += math.magnitude([p2.x-p1.x,p2.y-p1.y])-math.magnitude(this.particles[i].startDeltas[j]);
				// console.log(this.particles[i].startDeltas[j]);
			}
			jointsCount += this.particles[i].joints.length;
		}
		return deform/jointsCount;
	}
}
class BodyConstructor {
	constructor() {
		this.newParticles = [];
		this.firstJointParticle = null;
		this.isMouseHoveredOnBody = false;
		this.hoverTarget = null;

		this.jointsCount = 0;
	}
	AddParticle(Position, index = -1, isFreezed = false) {
		if (!this.isMouseHoveredOnBody) {
			let p = new Particle(Position, { x: 0, y: 0 }, 0.339, 0.13, 0.08);
			p.isFreezed = isFreezed;
			p.inGroupID = this.newParticles.length;
			if (index == -1)
				this.newParticles.push(p);
			else
				this.newParticles.splice(index, 0, p);
		}
	}
	AddJointTo(from, to) {
		from.joints.push(to);
		to.joints.push(from);
		from.recalculateDistances();
		to.recalculateDistances();

		this.jointsCount++;
	}
	CreateNewGroup() {
		let pId = pGroups.push(new ParticlesGroup(this.newParticles));
		let startParticlesCount = particles.length;
		for (let i = 0; i < this.newParticles.length; i++) {
			this.newParticles[i].appearFrame = frame;
			particles.push(this.newParticles[i]);
			particles[startParticlesCount + i].groupIndex = pId - 1;
			particles[startParticlesCount + i].groupLink = pGroups[pId - 1];
			particles[startParticlesCount + i].selfId = startParticlesCount + i;
			// console.log(startParticlesCount+i);
		}
		if (this.jointsCount == 0) {
			pGroups[pId - 1].RecreateJoints();
		}
		return true;
	}
	PreDraw() {
		this.isMouseHoveredOnBody = false;
		let centere = this.GetCentere();
		for (let i = 0; i < this.newParticles.length; i++) {
			if (math.distance([input.mouse.x, input.mouse.y], [this.newParticles[i].position.x, this.newParticles[i].position.y])[0] < 8) {
				d.circle(this.newParticles[i].position.x, this.newParticles[i].position.y, 8, "yellow", "yellow", 1, false);
				this.isMouseHoveredOnBody = true;
				this.hoverTarget = this.newParticles[i];
			}
			let nextI = i + 1;
			if (nextI > this.newParticles.length - 1) nextI = 0;
			d.circle(this.newParticles[i].position.x, this.newParticles[i].position.y, 5, "gray");
			d.circle(centere.x, centere.y, 1, "red");
			d.line(this.newParticles[i].position.x, this.newParticles[i].position.y, this.newParticles[nextI].position.x, this.newParticles[nextI].position.y, "white");
			// d.txt(i,this.newParticles[i].position.x,this.newParticles[i].position.y,"","white");
			for (let j = 0; j < this.newParticles[i].joints.length; j++) {
				d.line(this.newParticles[i].position.x, this.newParticles[i].position.y,
					this.newParticles[i].joints[j].position.x, this.newParticles[i].joints[j].position.y, "red");
			}
		}
	}
	GetCentere() {
		let pos = { x: 0, y: 0 };
		for (let i = 0; i < this.newParticles.length; i++) {
			// if (isNaN(this.newParticles[i].position)) console.log(i);
			pos.x += this.newParticles[i].position.x;
			pos.y += this.newParticles[i].position.y;
		}
		return { x: pos.x / this.newParticles.length, y: pos.y / this.newParticles.length };
	}
	Tesselate(OffsetFunc) { // arguments = (normal,p1Position,p2Position,centere) // return = (x,y)
		console.log(this.newParticles.length * 2);
		let partsPosesToAdd = [];
		let centere = this.GetCentere();
		for (let i = 0; i < this.newParticles.length; i++) {
			let nextI = i + 1;
			if (nextI > this.newParticles.length - 1) nextI = 0;
			let pos = {
				x: (this.newParticles[i].position.x + this.newParticles[nextI].position.x) / 2,
				y: (this.newParticles[i].position.y + this.newParticles[nextI].position.y) / 2
			};

			let deltaX = (this.newParticles[nextI].position.x - this.newParticles[i].position.x);
			let deltaY = (this.newParticles[nextI].position.y - this.newParticles[i].position.y);
			let offset = OffsetFunc({ x: -deltaY, y: deltaX }, this.newParticles[i].position, pos, this.newParticles[nextI].position, centere);
			pos.x += offset.x;
			pos.y += offset.y;
			partsPosesToAdd.push(pos);
		}
		for (let i = 0; i < partsPosesToAdd.length; i++) {
			let nextI = i + 1;
			if (nextI > this.newParticles.length - 1) nextI = 0;
			this.AddParticle(partsPosesToAdd[i], nextI + i);
		}
	}
}

var pGroups = [];
var particles = [];

var walls = [];
var wallsCount = 50;

var startY = canv.height / 2 + 200, startX = canv.width / 2;
var amplitude = 50, frequency = 16;

function eq(x) { // -1 -> 1
	return -x * x;
	// return x;

	return Math.sin(x * 0.7) * 1;
}
function getEqDeriative(x, offset = 0.01) {
	return (eq(x + offset) - eq(x)) / offset;
}
for (let i = -wallsCount / 2; i < wallsCount / 2; i += 0) {
	// eq = sin(x)
	let x1 = i / wallsCount * frequency;
	let x2 = (i + 1) / wallsCount * frequency;

	let p1y = eq(x1) * amplitude;
	let p2y = eq(x2) * amplitude;

	walls.push(new Wall({ x: startX + x1 * canv.width / frequency, y: startY + p1y }, { x: startX + x2 * canv.width / frequency, y: startY + p2y }));
	// i += 1+Math.abs(getEqDeriative(x1));
	i++;
}


var groupsCount = 0;
let r = 40;
let count = 30;
for (let p = 0; p < groupsCount; p++) {
	let newParticles = [];
	for (let i = 0; i < count; i++) {
		let localPos = { x: 100 + math.cos(360 / count * i) * r, y: math.sin(360 / count * i) * r };
		let velocity = { x: -math.sin(360 / count * i) * 0, y: math.cos(360 / count * i) * 0 };
		let particle = new Particle({ x: 450 + localPos.x, y: canv.height / 2 + localPos.y + 100 + p * 90 - 200 }, velocity, 0.02, 0.18, 0.5);
		// particle.group
		newParticles.push(particle);
		newParticles[newParticles.length - 1].groupIndex = p;

		particles.push(particle);
	}

	let l = pGroups.push(new ParticlesGroup(newParticles));
	pGroups[l - 1].RecreateJoints(2);
}
// for (let p = 0; p < groupsCount; p++) {
// 	let newParticles = [];
// 	let width = Math.sqrt(count);
// 	for (let i = 0; i < count; i++) {
// 		let pos = {
// 			x: 200+(i%width)*r,
// 			y: 200+Math.floor(i/width)*r
// 		};
// 		let a = Math.random()*2*Math.PI;
// 		let dst = 80+Math.random()*(10);
// 		pos = {
// 			x: 200+Math.cos(a)*dst + p * r*2.2,
// 			y: 200+Math.sin(a)*dst
// 		};
// 		let particle = new Particle(pos,{x:0,y:0},0.019,0,0.03);
// 		newParticles.push(particle);
// 		newParticles[newParticles.length-1].groupIndex = p;
// 		particles.push(particle);
// 	}
// 	let l = pGroups.push(new ParticlesGroup(newParticles));
// 	pGroups[l-1].RecreateJoints();
// }

var bodyConstruction;
canv.addEventListener("mousedown", (e) => {
	if (bodyConstruction != null) {
		if (e.buttons == 1) {
			if (!bodyConstruction.isMouseHoveredOnBody) {
				bodyConstruction.AddParticle({ x: e.offsetX, y: e.offsetY });
			} else {
				if (bodyConstruction.firstJointParticle != null && bodyConstruction.firstJointParticle != bodyConstruction.hoverTarget) {
					bodyConstruction.AddJointTo(bodyConstruction.firstJointParticle, bodyConstruction.hoverTarget);
					bodyConstruction.firstJointParticle = null;
				} else {
					bodyConstruction.firstJointParticle = bodyConstruction.hoverTarget;
				}

			}
		}
		if (e.buttons == 4) {
			if (!bodyConstruction.isMouseHoveredOnBody)
				bodyConstruction.AddParticle({ x: e.offsetX, y: e.offsetY }, -1, true);
		}
	}
});
function tesselFuncRounding(normal, p1, c, p2, centere) {
	let dst1 = math.magnitude({ x: p1.x - centere.x, y: p1.y - centere.y });
	let dst2 = math.magnitude({ x: c.x - centere.x, y: c.y - centere.y });
	let dir = math.normalize({ x: c.x - centere.x, y: c.y - centere.y });
	return { x: dir.x * dst1 - dir.x * dst2, y: dir.y * dst1 - dir.y * dst2 };
	return { x: normal.x * Math.random() * 0.04, y: normal.y * Math.random() * 0.04 };
}
function tesselFuncChaoting(normal, p1, c, p2, centere) {
	return { x: normal.x * (-1 + Math.random() * 2) * 0.05, y: normal.y * (-1 + Math.random() * 2) * 0.05 };
}
function tesselFuncEmpty(normal, p1, c, p2, centere) {
	return { x: 0, y: 0 };
}
document.addEventListener("wheel", (e) => {
	if (bodyConstruction != null) {
		bodyConstruction.Tesselate(tesselFuncEmpty);
	}
});

let targetDrag;
function drag() {
	// if (input.mouse.isMouseDown(1)) {
	// 	// console.log("1");
	// 	let minDistance = Infinity;
	// 	let minDIndex = 0;
	// 	for (let i = 0; i < particles.length; i++) {
	// 		let d = math.magnitude({x:particles[i].position.x-input.mouse.x,y:particles[i].position.y-input.mouse.y});
	// 		if (d < minDistance) {
	// 			minDistance = d;
	// 			minDIndex = i;
	// 		}
	// 	}
	// 	targetDrag = minDIndex;
	// }
	// if (input.mouse.click == 1 && bodyConstruction == null) {
	// 	particles[targetDrag].position = {x:input.mouse.x,y:input.mouse.y};
	// 	particles[targetDrag].velocity = {x:0,y:0};
	// }

	if (input.mouse.click == 1 && bodyConstruction == null) {
		for (let i = 0; i < particles.length; i++) {
			// particles[targetDrag].position = {x:input.mouse.x,y:input.mouse.y};
			let d = math.magnitude({ x: input.mouse.x - particles[i].position.x, y: input.mouse.y - particles[i].position.y });
			particles[i].velocity.x += (input.mouse.x - particles[i].position.x) * 50 / d / d;
			particles[i].velocity.y += (input.mouse.y - particles[i].position.y) * 50 / d / d;
		}
	}
}

var framesCountToRecord = Infinity;

var replayFrame = 0;

var slidOffset = 50;
var ReplaySlider = new Slider({ x: slidOffset, y: canv.height - 20 }, "", (v) => { replayFrame = Math.floor(v); }, 2, canv.width - slidOffset * 2, 0, 0, 200);

// TODO: distribute velocity of particle to all else particles
setInterval(function () {

	deltaTime = (new Date() - startTime);
	startTime = new Date();
	fps += 1000 / deltaTime;
	if (frame % 5 == 0) {
		rFps = fps / 5;
		fps = 0;
	}

	d.clear("black");
	if (input.keyboard.char == 'r') {
		framesCountToRecord = frame + 1;
		ReplaySlider.max = frame;
	}
	if (input.keyboard.char == 'g' && bodyConstruction == null) {
		bodyConstruction = new BodyConstructor();
	}
	if (bodyConstruction != null) {
		bodyConstruction.PreDraw();
		if (input.mouse.click == 3) {
			bodyConstruction.CreateNewGroup();
			bodyConstruction = null;
		}
	}
	if (frame < framesCountToRecord) {
		for (let i = 0; i < pGroups.length; i++) {
			pGroups[i].Draw();
			let centere = pGroups[i].GetCentere();
			pGroups[i].averageVelocity = pGroups[i].GetAverageVelocity();
			d.ray(centere.x, centere.y, pGroups[i].averageVelocity, 20, "red");
			d.rect(centere.x, centere.y, 2, 2, "red", "Red");

			if (i == 0) {
				// debugGraph.data[0].data.push(new GraphData((canv.height - centere.y) / debugGraph.height, frame - particles[i].appearFrame, inRgb(255, 0, 0), "Y Position"));
				debugGraph.data[0].data.push({x:frame,y:pGroups[i].GetAverageDeformationLevel()});
				debugGraph.data[1].data.push({x:frame,y:-pGroups[i].averageVelocity.y});
				// debugGraph.data[1].push(new GraphData(math.magnitude(pGroups[i].averageVelocity), frame - particles[i].appearFrame, inRgb(0, 255, 0), "Average velocity"));
			}
		}
		for (let time = 0; time < 1/dt; time++) {
			for (let i = 0; i < particles.length; i++) {
				let nextI = i + 1;
				if (nextI > particles.length - 1)
					nextI = 0;
				// particles[i].recalculateDistances(particles[i].position, true);
				// particles[i].deformStructure();
				particles[i].springByDistances();
			}
			for (let i = 0; i < particles.length; i++) {
				if (particles[i].isFreezed) continue;
				particles[i].position.x += particles[i].velocity.x * dt;
				particles[i].position.y += particles[i].velocity.y * dt;

				for (let wall = 0; wall < walls.length; wall++) {
					let interData = particles[i].isIntersectWithWall(walls[wall]);
					if (interData.isInter) {
						particles[i].reactOnWall(walls[wall], interData.error);
					}
				}
				for (let g = 0; g < pGroups.length; g++) {
					if (particles[i].groupIndex == g) continue;
					let collData = particles[i].IsIntersectWithGroup(pGroups[g]);
					if (collData.isInter) {
						particles[i].reactOnDynamicWall(collData.Wall, collData.error);
					}
				}
				// d.txt(particles[i].selfId,particles[i].position.x,particles[i].position.y+3,"","white");
			}
			for (let i = 0; i < particles.length; i++) {
				// particles[i].springByDistances();
				particles[i].velocity.y += 0.1 * dt;
				if (particles[i].position.y < 0) {
					particles[i].velocity.y = -particles[i].velocity.y * 0.8;
					particles[i].velocity.x *= 0.9;
					particles[i].position.y = 0;
				}
				if (particles[i].position.y > canv.height) {
					particles[i].velocity.y = -particles[i].velocity.y * 1;
					particles[i].position.y = canv.height;
					particles[i].velocity.x *= 1;
				}
				if (particles[i].position.x < 0) {
					particles[i].velocity.x = -particles[i].velocity.x * 0.8;
					particles[i].position.x = 0;
				}
				if (particles[i].position.x > canv.width) {
					particles[i].velocity.x = -particles[i].velocity.x * 0.8;
					particles[i].position.x = canv.width;
				}
			}
			drag();
		}

		d.txt(frame + " / " + framesCountToRecord, 1, 16 + 16, "", "white");
		// Recording.AddData();
	} else {
		let rParticles = [];
		let pastId = -1;
		for (let i = 0; i < Recording.recordedData[replayFrame].length; i++) {
			const currId = Recording.recordedData[replayFrame][i].id;

			if (pastId != currId || i == Recording.recordedData[replayFrame].length - 1) {
				let grp = new ParticlesGroup(rParticles);
				grp.Draw();
				rParticles = [];
			} else {
				rParticles.push({ position: Recording.recordedData[replayFrame][i].position });
			}
			pastId = currId;
		}
		// for (let i = 0; i < Recording.recordedData[replayFrame].length; i++) {
		// 	d.circle(Recording.recordedData[replayFrame][i].position.x,Recording.recordedData[replayFrame][i].position.y,3,"gray");
		// }
		ReplaySlider.control();
		ReplaySlider.draw();
		d.txt(replayFrame + " / " + framesCountToRecord, 1, 16 + 16, "", "white");
	}
	for (let wall = 0; wall < walls.length; wall++) {
		walls[wall].draw();
	}
	if (needDrawDebugGraph) {
		debugGraph.viewOffset[0] = debugGraph.LocalToGlobal(-100-debugGraph.data[0].data[debugGraph.data[0].data.length-1].x,0)[0]
		debugGraph.Draw();
	}
	d.txt(Math.round(rFps), 1, 16, "", "white");
	d.txt(input.mouse.x + ", " + input.mouse.y, 1, 16 + 16 * 2, "", "white");
	frame++;
}, 1000 / 60);