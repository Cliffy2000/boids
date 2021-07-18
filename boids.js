var boids = [];
var boidCount = 150;

/*
const width = window.innerWidth;
const height = window.innerHeight;
*/

var width;
var height;
var mouseX = width / 2;
var mouseY = height / 2;
var test = 1;

var v1 = -0.1;
var v2 = 0.1

class Boid {
  constructor() {
    this.size = 1;
    this.color = "#558cf4";

    this.x = Math.random() * width;
    this.y = Math.random() * height;
    this.vX = Math.random() * 6 - 3;
    this.vY = Math.random() * 6 - 3;
    this.v = Math.sqrt(this.vX**2 + this.vY**2);
    this.direction = Math.atan2(this.vY, this.vX);
    this.aX = Math.random() * 8 - 4;
    this.aY = Math.random() * 8 - 4;
    this.maxSpeed = 3;
    this.maxForce = 0.25;

    this.wanderCheck = false;
    this.wanderAngle = Math.random() * Math.PI * 2;

    this.group = 1;
    this.leader = false;
    this.influence = 1;
    this.influenceRange = 100;
    this.actionRange = 100;
  }

  draw(ctx) {
    let canvasX = this.x;
    let canvasY = this.y;

    ctx.translate(canvasX, canvasY);
    ctx.rotate(this.direction);
    ctx.translate(-canvasX, -canvasY);
    ctx.fillStyle = this.color;

    let size = Math.pow(this.size, 0.3);
    ctx.beginPath();
    ctx.moveTo(canvasX, canvasY);
    ctx.lineTo(canvasX - 13.5*size, canvasY + 4.5*size);
    ctx.lineTo(canvasX - 13.5*size, canvasY - 4.5*size);
    ctx.lineTo(canvasX, canvasY);
    
    ctx.fill();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
  }
  
  limitAcceleration() {
    let a = Math.sqrt(this.aX**2 + this.aY**2);
    if (a > this.maxForce) {
      this.aX *= (this.maxForce / a);
      this.aY *= (this.maxForce / a);
    }
  }

  update() {
    this.vX += this.aX;
    this.vY += this.aY;

    let v = Math.sqrt(this.vX**2 + this.vY**2);
    if (v > this.maxSpeed) {
      this.vX *= (this.maxSpeed / v);
      this.vY *= (this.maxSpeed / v);
    }

    this.x += this.vX;
    this.y += this.vY;
    this.v = Math.sqrt(this.vX**2 + this.vY**2);
    this.aX = 0;
    this.aY = 0;

    if (this.vX != 0 && this.vY != 0) {
      this.direction = Math.atan2(this.vY, this.vX);
    }

    [this.x, this.y] = bound(this.x, this.y);
  }

  seekFlee(targetX, targetY, mode=1, coeff=1) {
    // This changes the orientation of the velocity vector but not the magnitude
    // mode is 1 for seeking and -1 for fleeing
    let desiredX = targetX - this.x;
    let desiredY = targetY - this.y;
    let dist = Math.sqrt(desiredX**2 + desiredY**2);
    
    // the desired values are negated compared to original algorithm, 
    // thus the (steering = desired - current) step should be skipped
    this.aX += (desiredX * (this.v / dist) * mode) * coeff;
    this.aY += (desiredY * (this.v / dist) * mode) * coeff;
  }

  mouseSeekFlee(mode=1) {
    if (Math.sqrt((this.x - mouseX)**2 + (this.y - mouseY)**2) < this.actionRange) {
      this.seekFlee(mouseX, mouseY, mode, 3);
    }
  }

  arrive(targetX, targetY, coeff=1) {
    let desiredX = targetX - this.x;
    let desiredY = targetY - this.y;
    let dist = Math.sqrt(desiredX**2 + desiredY**2);

    let c = Math.min(dist / this.actionRange, 1) ** 0.75;
    if (c > 0.01) {
      this.aX += ((desiredX * (c / dist) * this.maxSpeed) - this.vX) * coeff;
      this.aY += ((desiredY * (c / dist) * this.maxSpeed) - this.vY) * coeff;
    } else {
      this.vX = 0;
      this.vY = 0;
    }
  }

  pursuit(target) {
    let dist = Math.sqrt((target.x - this.x)**2 + (target.y - this.y)**2);
    if (dist < this.actionRange) {
      this.align([target]);

    }
    let estimateT1 = dist / Math.sqrt((target.vX - this.vX)**2 + (target.vY - this.vY)**2);
    let estimateT2 = dist / (this.maxSpeed + target.maxSpeed);
    let estimateT = Math.min(estimateT1, estimateT2);

    let estimateX = target.x + estimateT * target.vX;
    let estimateY = target.y + estimateT * target.vY;
    this.seekFlee(estimateX, estimateY);
  }

  separate(group) {
    let steeringX = 0;
    let steeringY = 0;
    let count = 0
    let countInf = 0;

    for (let each of group) {
      let distX = each.x - this.x;
      let distY = each.y - this.y;
      let dist = Math.sqrt(distX**2 + distY**2);

      if (0 < dist && dist < this.actionRange) {
        count += 1;
        countInf += each.influence;
        steeringX += (distX / Math.sqrt(distX**2 + distY**2)) * each.influence;
        steeringY += (distY / Math.sqrt(distX**2 + distY**2)) * each.influence;
      }
    }

    if (count != 0) {
      this.aX += (this.vX - steeringX*count/countInf);
      this.aY += (this.vY - steeringY*count/countInf);
    }
  }

  cohere(group) {
    let avgX = 0;
    let avgY = 0;
    let count = 0;

    for (let each of group) {
      if (each.leader == false && each.group == this.group){
        let distX = each.x - this.x;
        let distY = each.y - this.y;
        let dist = Math.sqrt(distX**2 + distY**2);

        if (0 < dist && dist < this.actionRange) {
          count += each.influence;
          avgX += each.x * each.influence;
          avgY += each.y * each.influence;
        }
      }
    }

    if (count != 0) {
      this.seekFlee(avgX/count, avgY/count);
    }
  }

  align(group) {
    let avgvX = 0;
    let avgvY = 0;
    let count = 0;

    for (let each of group) {
      if (each.group == this.group) {
        let distX = each.x - this.x;
        let distY = each.y - this.y;
        let dist = Math.sqrt(distX**2 + distY**2);

        if (0 < dist && dist < this.actionRange) {
          count += each.influence;
          avgvX += each.vX * each.influence;
          avgvY += each.vY * each.influence;
        }
      } 
    }

    if (count != 0) {
      this.aX += (avgvX/count - this.vX);
      this.aY += (avgvY/count - this.vY);
    }
  }

  wander() {
    if (this.wanderCheck == true) {
      this.wanderAngle += (Math.random()*0.4 - 0.2);

      if (this.wanderAngle < -Math.PI) {
        this.wanderAngle += Math.PI;
      }
      if (this.wanderAngle > Math.PI) {
        this.wanderAngle -= Math.PI;
      }

      this.aX += Math.sin(this.wanderAngle);
      this.aY += Math.cos(this.wanderAngle);
    }
  }

  follow(leader) {
    let pathDist = pointLineDist(this.x, this.y, leader.x, leader.y, 
      leader.x + leader.vX*leader.influence*3, leader.y + leader.vY*leader.influence*3);

    if (pathDist != false && pathDist[0] < leader.influence*4) {
      this.color = "#000000";
      this.seekFlee(pathDist[1], pathDist[2], -1, leader.influence);
    } else {
      this.color = "#558cf4";
    }

    let dist1 = Math.sqrt((this.x - leader.x)**2 + (this.y - leader.y)**2);
    let posX = leader.x - leader.vX*leader.influence*2;
    let posY = leader.y - leader.vY*leader.influence*2;
    [posX, posY] = bound(posX, posY);
    let dist2 = Math.sqrt((this.x - posX)**2 + (this.y - posY)**2);

    if (dist1 < this.actionRange) {
      this.seekFlee(leader.x + leader.vX*leader.influence*2, leader.y + leader.vY*leader.influence*2, 1, leader.influence);
    } else if (dist2 < this.actionRange) {
      this.seekFlee(posX+leader.vX*leader.influence*2, posY+leader.vY*leader.influence*2, 1, leader.influence);
    }
  }
}


function pointLineDist(pX, pY, lX1, lY1, lX2, lY2) {
  // just in case the length of the line is 0
  if (lX1 == lX2 && lY1 == lY2) {
    return false;
  }

  let p1 = (pX - lX1)*(lX2 - lX1) + (pY - lY1)*(lY2 - lY1);
  let p2 = (lX2 - lX1)**2 + (lY2 - lY1)**2;
  let p = p1 / p2;

  if (p < 0) {
    //return Math.sqrt((pX - lX1)**2 + (pY - lY1)**2);
    return false;
  } else if (p > 1) {
    //return Math.sqrt((pX - lX2)**2 + (pY - lY1)**2);
    return false;
  } else {
    let posX = lX1 + p * (lX2 - lX1);
    let posY = lY1 + p * (lY2 - lY1);
    return [Math.sqrt((pX - posX)**2 + (pY - posY)**2), posX, posY];
  }
}


function bound(x, y) {
  if (x > width) {
    x -= width;
  } else if (x < 0) {
    x += width;
  }
  if (y > height) {
    y -= height;
  } else if (y < 0) {
    y += height;
  }

  return [x, y];
}


function spawnBoids() {
  for (let i=0; i < boidCount; i++) {
    boids[boids.length] = new Boid();
  }
}


function resizeCanvas() {
  const canvas = document.getElementById("Boids");
  canvas.width = width;
  canvas.height = height;
}


function updateMouse(event) {
  mouseX = event.pageX - document.getElementById("Boids").offsetLeft;
  mouseY = event.pageY - document.getElementById("Boids").offsetTop;
}


function testChange() {
  test *= -1;
}


function nextFrame() {
  for (let i=1; i<boidCount; i++) {
    boids[i].follow(boids[0]);

    boids[i].separate(boids);
    boids[i].cohere(boids);
    boids[i].align(boids);
    boids[i].mouseSeekFlee(mode=test);
  }

  boids[0].wander();


  for (let b of boids) {
    /*
    b.separate(boids);
    b.cohere(boids);
    b.align(boids);
    b.mouseSeekFlee(mode=test);
    */
    //b.wander();

    b.limitAcceleration();
    b.update();
  }

  const ctx = document.getElementById("Boids").getContext("2d");
  ctx.clearRect(0, 0, width, height);

  for (let b of boids) {
    b.draw(ctx);
  }

  document.getElementById("rangeText").innerHTML = document.getElementById("range").value;
  window.requestAnimationFrame(nextFrame);
}


window.onload = () => {
  width = document.getElementById("testDiv").offsetWidth;
  height = window.innerHeight - 150;
  // this is to make the canvas fullscreen
  //window.addEventListener("resize", resizeCanvas, false); 
  resizeCanvas();

  spawnBoids();
  boids[0].color = "#ff0000";
  boids[0].size = 4;
  boids[0].wanderCheck = true;
  boids[0].influence = 20;
  boids[0].maxForce = 0.15;
  boids[0].leader = true;

  var mouse = document.querySelector("#Boids");
  mouse.addEventListener("mousemove", updateMouse, false);
  mouse.addEventListener("mouseenter", updateMouse, false);
  mouse.addEventListener("mouseleave", updateMouse, false);

  window.requestAnimationFrame(nextFrame);
}
