var boids = [];
var boidCount = 2;

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
    this.maxSpeed = 4;
    this.maxForce = 0.25;
    this.influence = 1;
    this.influenceRange = 100;
    this.actionRange = 100;

    this.wanderCheck = false;
    this.wanderAngle = Math.random() * Math.PI * 2;
  }

  draw(ctx) {
    let canvasX = this.x;
    let canvasY = this.y;

    ctx.translate(canvasX, canvasY);
    ctx.rotate(this.direction);
    ctx.translate(-canvasX, -canvasY);
    ctx.fillStyle = this.color;

    ctx.beginPath();
    ctx.moveTo(canvasX, canvasY);
    ctx.lineTo(canvasX - 15, canvasY + 5);
    ctx.lineTo(canvasX - 15, canvasY - 5);
    ctx.lineTo(canvasX, canvasY);
    
    ctx.fill();
    ctx.setTransform(1, 0, 0, 1, 0, 0);
  }

  bound() {
    if (this.x > width) {
      this.x -= width;
    }
    if (this.x < 0) {
      this.x += width;
    }
    if (this.y > height) {
      this.y -= height;
    }
    if (this.y < 0) {
      this.y += height;
    }
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
  }

  seekFlee(targetX, targetY, mode=1) {
    // This changes the orientation of the velocity vector but not the magnitude
    // mode is 1 for seeking and -1 for fleeing
    let desiredX = targetX - this.x;
    let desiredY = targetY - this.y;
    let dist = Math.sqrt(desiredX**2 + desiredY**2);
    
    // the desired values are negated compared to original algorithm, 
    // thus the (steering = desired - current) step should be skipped
    this.aX += (desiredX * (this.v / dist) * mode);
    this.aY += (desiredY * (this.v / dist) * mode);
  }

  mouseSeekFlee(mode=1) {
    if (Math.sqrt((this.x - mouseX)**2 + (this.y - mouseY)**2) < this.actionRange) {
      this.seekFlee(mouseX, mouseY, mode=mode);
      this.seekFlee(mouseX, mouseY, mode=mode);
      this.seekFlee(mouseX, mouseY, mode=mode);
    }
  }

  arrive(targetX, targetY) {
    let desiredX = targetX - this.x;
    let desiredY = targetY - this.y;
    let dist = Math.sqrt(desiredX**2 + desiredY**2);

    let coeff = Math.min(dist / this.actionRange, 1) ** 0.75;
    if (coeff > 0.01) {
      this.aX += ((desiredX * (coeff / dist) * this.maxSpeed) - this.vX);
      this.aY += ((desiredY * (coeff / dist) * this.maxSpeed) - this.vY);
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

    for (let each of group) {
      let distX = each.x - this.x;
      let distY = each.y - this.y;
      let dist = Math.sqrt(distX**2 + distY**2);

      if (0 < dist && dist < this.actionRange) {
        steeringX += (distX / Math.sqrt(distX**2 + distY**2));
        steeringY += (distY / Math.sqrt(distX**2 + distY**2));
      }
      
    }

    this.aX += (this.vX - steeringX);
    this.aY += (this.vY - steeringY);
  }

  cohere(group) {
    let avgX = 0;
    let avgY = 0;
    let count = 0;

    for (let each of group) {
      let distX = each.x - this.x;
      let distY = each.y - this.y;
      let dist = Math.sqrt(distX**2 + distY**2);

      if (0 < dist && dist < this.actionRange) {
        count += each.influence;
        avgX += each.x * each.influence;
        avgY += each.y * each.influence;
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
      let distX = each.x - this.x;
      let distY = each.y - this.y;
      let dist = Math.sqrt(distX**2 + distY**2);

      if (0 < dist && dist < this.actionRange) {
        count += each.influence;
        avgvX += each.vX * each.influence;
        avgvY += each.vY * each.influence;
      }
    }

    if (count != 0) {
      this.aX += (avgvX/count - this.vX);
      this.aY += (avgvY/count - this.vY);
    }
  }

  wander() {
    if (this.wanderCheck == true) {
      this.wanderAngle += (Math.random()*0.5 - 0.25);

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
    let dist = pointLineDist(this.x, this.y, leader.x, leader.y, leader.x + leader.vX*40, leader.y + leader.vY*40);
    console.log(leader.x-this.x, dist)

  }
}


function pointLineDist(pX, pY, lX1, lY1, lX2, lY2) {
  // just in case the length of the line is 0
  if (lX1 == lX2 && lY1 == lY2) {
    return Math.sqrt((pX - lX1)**2 + (pY - lY1)**2);
  }

  let p1 = (pX - lX1)*(lX2 - lX1) + (pY - lY1)*(lY2 - lY1);
  let p2 = (lX2 - lX1)**2 + (lY2 - lY1)**2;
  let p = p1 / p2;

  if (p < 0) {
    console.log('p < 0', p);
    return Math.sqrt((pX - lX1)**2 + (pY - lY1)**2);
  } else if (p > 1) {
    console.log('p > 1', p);
    return Math.sqrt((pX - lX2)**2 + (pY - lY1)**2);
  } else {
    console.log('0 < p < 1', p);
    return Math.sqrt((pX - (lX1 + p * (lX2 - lX1)))**2 + (pY - (lY1 + p * (lY2 - lY1)))**2);
  }
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
  boids[1].follow(boids[0]);
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
    b.bound();
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

  boids[0].x = 200;
  boids[0].y = 400;
  boids[0].vX = 1;
  boids[0].vY = 0;
  boids[0].aX = 0;
  boids[0].aY = 0;
  boids[1].x = 500;
  boids[1].y = 400;
  boids[1].vX = 0;
  boids[1].vY = 0;
  boids[1].aX = 0;
  boids[1].aY = 0;

  var mouse = document.querySelector("#Boids");
  mouse.addEventListener("mousemove", updateMouse, false);
  mouse.addEventListener("mouseenter", updateMouse, false);
  mouse.addEventListener("mouseleave", updateMouse, false);

  window.requestAnimationFrame(nextFrame);
}
