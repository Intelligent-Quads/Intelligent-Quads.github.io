droneImage = new Image();
droneImage.src = "drone.png";
droneWidth = 200;
droneHeight = 200;
droneCenter = { x: droneWidth / 2, y: droneHeight / 2 }
rotationCenter__droneFrame = { x: droneCenter.x, y: droneCenter.y }
angle = 0;


var csv = document.getElementById('canvas');
var ctx = csv.getContext('2d');
console.log(csv.width)

var rateGains = { p: 60, i: 1, iClamp: 100, d: 1 };
var attGains = { p: 5, i: 0, iClamp: 100, d: 0 };
var velGains = { p: 1, i: 0, iClamp: 100, d: 0 };
var posGains = { p: 1, i: 0, iClamp: 100, d: 0 };

var angle_param = 45; // deg
var speed_param = 5; // m/s

var timeStep = 10; //ms
disturbance = {
    stepLength: 0, // ms
    stepAmp: 0 // N-m
}
disturbanceIC = {
    stepLength: 50, // ms
    stepAmp: 40 // N-m
}

desired = {
    desiredRate: 0,
    desiredAtt: 0,
    desiredVel: 0,
    desiredPos: 0
}

droneState = {
    alpha: 0,
    omega: 200,
    theta: 0,
    acceleration: 0,
    velocity: 0,
    position: 0
}

motor = {
    minRPM: 0,
    maxRPM: 5000,
    power2rpm: 4,
    rpm2force: .0001
}

drone = {
    mass: 2, //kg 
    inertia: .001, // kg-m^2
    armLength: 1, //m
    motorStrength: 3, //N
    hoverRPM: 200
}
posError = {
    pErr: 0,
    iErr: 0,
    dErr: 0
}
velError = {
    pErr: 0,
    iErr: 0,
    dErr: 0
}
attError = {
    pErr: 0,
    iErr: 0,
    dErr: 0
}
rateError = {
    pErr: 0,
    iErr: 0,
    dErr: 0
}

function updateDisplay() {
    document.getElementById("stateOmega").innerHTML = "Omega (deg/s): " + droneState.omega.toFixed(2).toString();
    document.getElementById("stateTheta").innerHTML = " Theta (deg): " + droneState.theta.toFixed(2).toString();
    document.getElementById("stateAccel").innerHTML = " Acceleration (deg): " + droneState.acceleration.toFixed(2).toString();
    document.getElementById("stateVel").innerHTML = " Velocity (m/s): " + droneState.velocity.toFixed(2).toString();
    document.getElementById("statePos").innerHTML = " Position (m): " + droneState.position.toFixed(2).toString();
}
function clamping(max, value) {
    if (value > max) {
        value = max;
        // console.log(value, "greator than max", max)
    }
    if (value < -max) {
        value = -max;
        // console.log(value, "greator than max", max)
    }
    return value;
}

function rateLoop() {
    currentError = desired.desiredRate - droneState.omega;
    rateError.dErr = (currentError - rateError.pErr) / (timeStep / 1000);
    rateError.pErr = currentError;
    rateError.iErr = clamping(rateGains.iClamp, currentError + rateError.iErr);
    // console.log(rateError)

    output = rateGains.p * rateError.pErr + rateGains.i * rateError.iErr + rateGains.d * rateError.dErr;
    // console.log(output)
    return output;
}
function attitudeLoop() {

    // apply angle limits
    if (desired.desiredAtt > angle_param)
        desired.desiredAtt = angle_param;
    else if (desired.desiredAtt < -angle_param)
        desired.desiredAtt = -angle_param;
    else
        desired.desiredAtt = desired.desiredAtt;

    // apply angle control loop in quickest direction and handle discontinuity
    currentError = desired.desiredAtt - droneState.theta;
    if (currentError > 180)
        currentError = currentError - 360;
    else if (currentError < -180)
        currentError = currentError + 360;
    else
        currentError = currentError;//do nothing

    attError.dErr = (currentError - attError.pErr) / (timeStep / 1000);
    attError.pErr = currentError;
    attError.iErr = clamping(attGains.iClamp, currentError + attError.iErr);
    // console.log(attError)

    output = attGains.p * attError.pErr + attGains.i * attError.iErr + attGains.d * attError.dErr;
    desired.desiredRate = output;
    output = rateLoop();

    return output;
}
function velocityLoop() {
    // apply angle limits
    if (desired.desiredVel > speed_param)
        desired.desiredVel = speed_param;
    else if (desired.desiredVel < -speed_param)
        desired.desiredVel = -speed_param;
    else
        desired.desiredVel = desired.desiredVel;

    currentError = desired.desiredVel - droneState.velocity;
    velError.dErr = (currentError - velError.pErr) / (timeStep / 1000);
    velError.pErr = currentError;
    velError.iErr = clamping(velGains.iClamp, currentError + velError.iErr);

    output = velGains.p * velError.pErr + velGains.i * velError.iErr + velGains.d * velError.dErr;
    // console.log("vel loop", output)
    desired.desiredAtt = output;
    output = attitudeLoop();
    // console.log("rate loop", output)
    return output;

}
function positionLoop() {
    currentError = desired.desiredPos - droneState.position;
    posError.dErr = (currentError - posError.pErr) / (timeStep / 1000);
    posError.pErr = currentError;
    posError.iErr = clamping(posGains.iClamp, currentError + posError.iErr);

    output = posGains.p * posError.pErr + posGains.i * posError.iErr + posGains.d * posError.dErr;
    // console.log("pos loop", output)
    desired.desiredVel = output;
    output = velocityLoop();
    return output;
}

function inRange(min, max, value) {
    if (value > max) {
        output = max;
    } else if (value < min) {
        output = min;
    } else {
        output = value;
    }
    return output;
}


function physics() {
    // calc control loops 
    if (document.getElementById("posLoopChBx").checked) {
        controlOutput = positionLoop();
    } else {
        if (document.getElementById("velLoopChBx").checked) {
            controlOutput = velocityLoop();
        } else {
            if (document.getElementById("attLoopChBx").checked) {
                controlOutput = attitudeLoop();
            } else {
                if (document.getElementById("rateLoopChBx").checked) {
                    controlOutput = rateLoop()
                } else {
                    console.log("no control loop active")
                    controlOutput = 0;
                }
            }
        }
    }


    // console.log("control output", controlOutput)

    // calc Forces 
    var motorForce1 = inRange(motor.minRPM, motor.maxRPM, drone.hoverRPM - (motor.rpm2force * controlOutput));
    var motorForce2 = inRange(motor.minRPM, motor.maxRPM, drone.hoverRPM + (motor.rpm2force * controlOutput));
    var motorForce = motorForce1 + motorForce2;
    var motorTorque = motorForce2 * drone.armLength - motorForce1 * drone.armLength;

    var rho = 1.225; // kg/m^3
    var dargDir = 1;
    if (droneState.velocity < 0) {
        dargDir = -1;
    }
    var dargForce = .5 * rho * (droneState.velocity ^ 2) * 0.6 * .1 * dargDir;

    droneState.alpha = (motorTorque + disturbance.stepAmp) / drone.inertia;

    droneState.omega = droneState.alpha * (timeStep / 1000) + droneState.omega;

    droneState.theta = (((droneState.omega * (timeStep / 1000) + droneState.theta) + 180) % 360) - 180;

    droneState.acceleration = ((motorForce * Math.sin(droneState.theta * (Math.PI / 180)) - dargForce) / drone.mass) //- ((droneState.velocity ^ 2) / drone.mass);

    droneState.velocity = droneState.acceleration * (timeStep / 1000) + droneState.velocity;

    droneState.position = droneState.velocity * (timeStep / 1000) + droneState.position;
    // console.log("motor force", motorForce);
    // console.log("drag force", dargForce);

}

function drawDrone(ctx, img, x, y, angle) {
    ctx.translate(x, y);
    ctx.rotate((Math.PI / 180) * angle);
    ctx.translate(-rotationCenter__droneFrame.x, -rotationCenter__droneFrame.y);
    ctx.drawImage(img, 0, 0, droneWidth, droneHeight);
    ctx.translate(rotationCenter__droneFrame.x, rotationCenter__droneFrame.y);
    ctx.rotate(-(Math.PI / 180) * angle);
    ctx.translate(-x, -y);
}
function animate(image) {
    canvasWidth = csv.width;
    canvasHeight = csv.height;
    physics();
    ctx.clearRect(0, 0, canvasWidth, canvasHeight);
    drawDrone(ctx, image, canvasHeight / 2, canvasWidth / 2, droneState.theta);
    updateDisplay();
}

function updateGains() {
    //Rate Loop
    if (isNaN(parseFloat(document.getElementById("ratePGain").value))) {
        rateGains.p = 0;
    } else {

        rateGains.p = parseFloat(document.getElementById("ratePGain").value);
    }
    if (isNaN(parseFloat(document.getElementById("rateIGain").value))) {
        rateGains.i = 0;
    } else {
        rateGains.i = parseFloat(document.getElementById("rateIGain").value);
    }
    if (isNaN(parseFloat(document.getElementById("rateMaxI").value))) {
        rateGains.iClamp = 0;
    } else {
        rateGains.iClamp = parseFloat(document.getElementById("rateMaxI").value);
    }
    if (isNaN(parseFloat(document.getElementById("rateDGain").value))) {
        rateGains.d = 0;
    } else {
        rateGains.d = parseFloat(document.getElementById("rateDGain").value);
    }
    console.log(rateGains)

    // ATTITUDE LOOP
    if (isNaN(parseFloat(document.getElementById("attPGain").value))) {
        attGains.p = 0;
    } else {
        attGains.p = parseFloat(document.getElementById("attPGain").value);
    }
    if (isNaN(parseFloat(document.getElementById("attIGain").value))) {
        attGains.i = 0;
    } else {
        attGains.i = parseFloat(document.getElementById("attIGain").value);
    }
    if (isNaN(parseFloat(document.getElementById("attMaxI").value))) {
        attGains.iClamp = 0;
    } else {
        attGains.iClamp = parseFloat(document.getElementById("attMaxI").value);
    }
    if (isNaN(parseFloat(document.getElementById("attDGain").value))) {
        attGains.d = 0;
    } else {
        attGains.d = parseFloat(document.getElementById("attDGain").value);
    }
    if (isNaN(parseFloat(document.getElementById("maxAngle").value))) {
        angle_param = 0;
    } else {
        angle_param = parseFloat(document.getElementById("maxAngle").value);
    }

    //VELOCITY LOOP
    if (isNaN(parseFloat(document.getElementById("velPGain").value))) {
        velGains.p = 0;
    } else {
        velGains.p = parseFloat(document.getElementById("velPGain").value);
    }

    if (isNaN(parseFloat(document.getElementById("velIGain").value))) {
        velGains.i = 0;
    } else {
        velGains.i = parseFloat(document.getElementById("velIGain").value);
    }
    if (isNaN(parseFloat(document.getElementById("velMaxI").value))) {
        velGains.iClamp = 0;
    } else {
        velGains.iClamp = parseFloat(document.getElementById("velMaxI").value);
    }
    if (isNaN(parseFloat(document.getElementById("velDGain").value))) {
        velGains.d = 0;
    } else {
        velGains.d = parseFloat(document.getElementById("velDGain").value);
    }
    if (isNaN(parseFloat(document.getElementById("maxSpeed").value))) {
        speed_param = 0;
    } else {
        speed_param = parseFloat(document.getElementById("maxSpeed").value);
    }

    //POSITION LOOP
    if (isNaN(parseFloat(document.getElementById("posPGain").value))) {
        posGains.p = 0;
    } else {
        posGains.p = parseFloat(document.getElementById("posPGain").value);
    }
    if (isNaN(parseFloat(document.getElementById("posIGain").value))) {
        posGains.i = 0;
    } else {
        posGains.i = parseFloat(document.getElementById("posIGain").value);
    }
    if (isNaN(parseFloat(document.getElementById("posMaxI").value))) {
        posGains.iClamp = 0;
    } else {
        posGains.iClamp = parseFloat(document.getElementById("posMaxI").value);
    }
    if (isNaN(parseFloat(document.getElementById("posDGain").value))) {
        posGains.d = 0;
    } else {
        posGains.d = parseFloat(document.getElementById("posDGain").value);
    }

    console.log(posGains)
    console.log(rateGains)
    console.log(attGains)
    return false;
}

function updateDesired() {
    if (isNaN(parseFloat(document.getElementById("desiredRate").value))) {
        desired.desiredRate = 0;
    } else {
        desired.desiredRate = parseFloat(document.getElementById("desiredRate").value);
    }

    if (isNaN(parseFloat(document.getElementById("desiredAtt").value))) {
        desired.desiredAtt = 0;
    } else {
        desired.desiredAtt = parseFloat(document.getElementById("desiredAtt").value);
    }

    if (isNaN(parseFloat(document.getElementById("desiredVel").value))) {
        desired.desiredVel = 0;
    } else {
        desired.desiredVel = parseFloat(document.getElementById("desiredVel").value);
    }

    if (isNaN(parseFloat(document.getElementById("desiredPos").value))) {
        desired.desiredPos = 0;
    } else {
        desired.desiredPos = parseFloat(document.getElementById("desiredPos").value);
    }

    console.log(desired)
    return false;
}
function updateDisturbance() {
    if (isNaN(parseFloat(document.getElementById("stepLength").value))) {
        disturbance.stepLength = 0;
    } else {
        disturbance.stepLength = parseFloat(document.getElementById("stepLength").value);
    }
    if (isNaN(parseFloat(document.getElementById("stepAmp").value))) {
        disturbance.stepAmp = 0;
        console.log("is nan")
    } else {
        disturbance.stepAmp = parseFloat(document.getElementById("stepAmp").value);
    }
    setTimeout(() => {
        console.log(disturbance)
        disturbance.stepAmp = 0;
        console.log(disturbance)
    }, disturbance.stepLength);

    return false;
}
function updateDrone() {
    console.log("updating drone");
    if (isNaN(parseFloat(document.getElementById("mass").value))) {
        document.getElementById("mass").value = drone.mass;
    } else {
        drone.mass = document.getElementById("mass").value
    }
    if (isNaN(parseFloat(document.getElementById("armLength").value))) {
        document.getElementById("armLength").value = drone.armLength;
    } else {
        drone.armLength = document.getElementById("armLength").value
    }
    if (isNaN(parseFloat(document.getElementById("inertiaDist").value))) {
        document.getElementById("inertiaDist").value = 0.5;
    } else {
        dist = document.getElementById("inertiaDist").value;
        //weighted moment of inertia about Iz of cuboid
        //lower dist -> drone is closer to point mass
        drone.inertia = (1 / 12) * drone.mass * ((.1) ^ 2 + (2 * drone.armLength * dist) ^ 2)
    }
    if (isNaN(parseFloat(document.getElementById("motStrength").value))) {
        document.getElementById("motStrength").value = motor.rpm2force * 1000;
    } else {
        motor.rpm2force = document.getElementById("motStrength").value / 1000;
    }
    return false;
}

function disableDesiredInput() {
    document.getElementById("desiredRate").disabled = true;
    document.getElementById("desiredAtt").disabled = true;
    document.getElementById("desiredVel").disabled = true;
    document.getElementById("desiredPos").disabled = true;
}

function enableRate(enableDisable) {
    document.getElementById("ratePGain").disabled = !enableDisable;
    document.getElementById("rateIGain").disabled = !enableDisable;
    document.getElementById("rateMaxI").disabled = !enableDisable;
    document.getElementById("rateDGain").disabled = !enableDisable;
    document.getElementById("desiredRate").disabled = !enableDisable;
    document.getElementById("rateLoopChBx").checked = enableDisable;

    document.getElementById("desiredRate").disabled = !enableDisable;
    document.getElementById("desiredAtt").disabled = enableDisable;
    document.getElementById("desiredVel").disabled = enableDisable;
    document.getElementById("desiredPos").disabled = enableDisable;

}
function enableAtt(enableDisable) {
    document.getElementById("attPGain").disabled = !enableDisable;
    document.getElementById("attIGain").disabled = !enableDisable;
    document.getElementById("attMaxI").disabled = !enableDisable;
    document.getElementById("attDGain").disabled = !enableDisable;
    document.getElementById("maxAngle").disabled = !enableDisable;
    document.getElementById("attLoopChBx").checked = enableDisable;

    document.getElementById("desiredRate").disabled = enableDisable;
    document.getElementById("desiredAtt").disabled = !enableDisable;
    document.getElementById("desiredVel").disabled = enableDisable;
    document.getElementById("desiredPos").disabled = enableDisable;
}
function enableVel(enableDisable) {
    document.getElementById("velPGain").disabled = !enableDisable;
    document.getElementById("velIGain").disabled = !enableDisable;
    document.getElementById("velMaxI").disabled = !enableDisable;
    document.getElementById("velDGain").disabled = !enableDisable;
    document.getElementById("maxSpeed").disabled = !enableDisable;
    document.getElementById("velLoopChBx").checked = enableDisable;

    document.getElementById("desiredRate").disabled = enableDisable;
    document.getElementById("desiredAtt").disabled = enableDisable;
    document.getElementById("desiredVel").disabled = !enableDisable;
    document.getElementById("desiredPos").disabled = enableDisable;
}

function enablePos(enableDisable) {
    document.getElementById("posPGain").disabled = !enableDisable;
    document.getElementById("posIGain").disabled = !enableDisable;
    document.getElementById("posMaxI").disabled = !enableDisable;
    document.getElementById("posDGain").disabled = !enableDisable;
    document.getElementById("posLoopChBx").checked = enableDisable;

    document.getElementById("desiredRate").disabled = enableDisable;
    document.getElementById("desiredAtt").disabled = enableDisable;
    document.getElementById("desiredVel").disabled = enableDisable;
    document.getElementById("desiredPos").disabled = !enableDisable;
}

function updateLoopsEnabled() {
    if (document.getElementById("posLoopChBx").checked) {
        enableRate(true);
        enableAtt(true);
        enableVel(true);
        enablePos(true);
    } else {
        if (document.getElementById("velLoopChBx").checked) {
            enableRate(true);
            enableAtt(true);
            enableVel(true);
        } else {
            if (document.getElementById("attLoopChBx").checked) {
                enableRate(true);
                enableAtt(true);
            } else {
                enableAtt(false);
                if (document.getElementById("rateLoopChBx").checked) {
                    enableRate(true);
                } else {
                    enableRate(false);
                    enableAtt(false);
                    disableDesiredInput();
                    // enableVel(false);
                    // enablePos(false);
                }
            }
        }
    }
}

function initDefaults() {
    document.getElementById("ratePGain").value = rateGains.p;
    document.getElementById("rateIGain").value = rateGains.i;
    document.getElementById("rateMaxI").value = rateGains.iClamp;
    document.getElementById("rateDGain").value = rateGains.d;

    document.getElementById("attPGain").value = attGains.p;
    document.getElementById("attIGain").value = attGains.i;
    document.getElementById("attMaxI").value = attGains.iClamp;
    document.getElementById("attDGain").value = attGains.d;
    document.getElementById("maxAngle").value = angle_param;

    document.getElementById("velPGain").value = velGains.p;
    document.getElementById("velIGain").value = velGains.i;
    document.getElementById("velMaxI").value = velGains.iClamp;
    document.getElementById("velDGain").value = velGains.d;
    document.getElementById("maxSpeed").value = speed_param;

    document.getElementById("posPGain").value = posGains.p;
    document.getElementById("posIGain").value = posGains.i;
    document.getElementById("posMaxI").value = posGains.iClamp;
    document.getElementById("posDGain").value = posGains.d;

    document.getElementById("stepAmp").value = disturbanceIC.stepAmp;
    document.getElementById("stepLength").value = disturbanceIC.stepLength;

    document.getElementById("desiredRate").value = desired.desiredRate;
    document.getElementById("desiredAtt").value = desired.desiredAtt;
    document.getElementById("desiredVel").value = desired.desiredVel;
    document.getElementById("desiredPos").value = desired.desiredPos;

    document.getElementById("mass").value = drone.mass;
    document.getElementById("armLength").value = drone.armLength;
    document.getElementById("inertiaDist").value = .5;
    document.getElementById("motStrength").value = motor.rpm2force * 1000;

    enableRate(false);
    enableAtt(false);
    enableVel(false);
    enablePos(false);
    disableDesiredInput();
}

initDefaults();
setInterval(() => {
    animate(droneImage);
}, timeStep);
