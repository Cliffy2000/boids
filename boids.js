var boids = [];
var boidCount = 100;

const width = window.innerWidth;
const height = window.innerHeight;

var mouseX = width / 2;
var mouseY = height / 2;


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
    this.maxSpeed = 5;
    this.maxForce = 0.4;
    this.viewRange = 100;
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
    if (Math.sqrt((this.x - mouseX)**2 + (this.y - mouseY)**2) < this.viewRange) {
      this.seekFlee(mouseX, mouseY, mode=mode);
      this.seekFlee(mouseX, mouseY, mode=mode);
      this.seekFlee(mouseX, mouseY, mode=mode);
    }
  }

  arrive(targetX, targetY) {
    let desiredX = targetX - this.x;
    let desiredY = targetY - this.y;
    let dist = Math.sqrt(desiredX**2 + desiredY**2);

    let coeff = Math.min(dist / this.viewRange, 1) ** 0.75;
    if (coeff > 0.01) {
      this.aX += ((desiredX * (coeff / dist) * this.maxSpeed) - this.vX);
      this.aY += ((desiredY * (coeff / dist) * this.maxSpeed) - this.vY);
    } else {
      this.vX = 0;
      this.vY = 0;
    }
  }

  separate(group) {
    let steeringX = 0;
    let steeringY = 0;

    for (let each of group) {
      let distX = each.x - this.x;
      let distY = each.y - this.y;
      let dist = Math.sqrt(distX**2 + distY**2);

      if (0 < dist && dist < this.viewRange) {
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

      if (0 < dist && dist < this.viewRange) {
        count += 1;
        avgX += each.x;
        avgY += each.y;
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

      if (0 < dist && dist < this.viewRange) {
        count += 1;
        avgvX += each.vX;
        avgvY += each.vY;
      }
    }

    if (count != 0) {
      this.aX += (avgvX/count - this.vX);
      this.aY += (avgvY/count - this.vY);
    }
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
  mouseX = event.pageX;
  mouseY = event.pageY;
}


function nextFrame() {
  for (let b of boids) {
    b.separate(boids);
    b.cohere(boids);
    b.align(boids);
    b.mouseSeekFlee(mode=1);

    b.limitAcceleration();
    b.update();
    b.bound();
  }

  const ctx = document.getElementById("Boids").getContext("2d");
  ctx.clearRect(0, 0, width, height);

  for (let b of boids) {
    b.draw(ctx);
  }

  window.requestAnimationFrame(nextFrame);
}


window.onload = () => {
  window.addEventListener("resize", resizeCanvas, false);
  resizeCanvas();

  spawnBoids();

  var mouse = document.querySelector("#Boids");
  mouse.addEventListener("mousemove", updateMouse, false);
  mouse.addEventListener("mouseenter", updateMouse, false);
  mouse.addEventListener("mouseleave", updateMouse, false);

  window.requestAnimationFrame(nextFrame);
}
