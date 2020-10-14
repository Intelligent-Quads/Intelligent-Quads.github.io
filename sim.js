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

var rateGains = { p: 60, i: 1, d: 1 };
var attGains = { p: 5, i: 0, d: 0 };
var velGains = { p: 0, i: 0, d: 0 };
var posGains = { p: 0, i: 0, d: 0 };


var timeStep = 5; //ms
disturbance = {
    stepLength: 0,
    stepAmp: 0
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
    rpm2force: 2
}

drone = {
    mass: 10, //kg 
    inertia: 5, // kg-m^2
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

// Specify the left and right ranges
var leftRange = [0, 100];
var rightRange = [-100, 100];

var lineChartData = [
    // First series
    {
        label: "Series 1",
        values: [{ time: 0, y: 1 }],
        range: rightRange
    }


    // // The second series
    // {
    //     label: "Series 2",
    //     values: [{ time: 1370044800, y: 78 }, { time: 1370044801, y: 98 }]
    // },

];
var graph = $('#lineChart').epoch({
    type: 'time.line',
    data: lineChartData
});

function updateDisplay() {
    document.getElementById("stateOmega").innerHTML = "Omega (deg/s): " + droneState.omega.toFixed(2).toString();
    document.getElementById("stateTheta").innerHTML = " Theta (deg): " + droneState.theta.toFixed(2).toString();
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
    rateError.iErr = clamping(100, currentError + rateError.iErr);
    // console.log(rateError)

    output = rateGains.p * rateError.pErr + rateGains.i * rateError.iErr + rateGains.d * rateError.dErr;
    // console.log(output)
    return output;
}
function attitudeLoop() {
    currentError = desired.desiredAtt - droneState.theta;
    attError.dErr = (currentError - attError.pErr) / (timeStep / 1000);
    attError.pErr = currentError;
    attError.iErr = clamping(100, currentError + attError.iErr);
    // console.log(attError)

    output = attGains.p * attError.pErr + attGains.i * attError.iErr + attGains.d * attError.dErr;
    desired.desiredRate = output;
    output = rateLoop();

    return output;
}
function velocityLoop() {
    currentError = desired.desiredVel - droneState.velocity;
    velError.dErr = (currentError - velError.pErr) / (timeStep / 1000);
    velError.pErr = currentError;
    velError.iErr = clamping(100, currentError + velError.iErr);

    output = velGains.p * velError.pErr + velGains.i * velError.iErr + velGains.d * velError.dErr;
    console.log("vel loop", output)
    desired.desiredAtt = output;
    output = attitudeLoop();
    output = rateLoop();
    console.log("rate loop", output)
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

    // console.log("control output", controlOutput)

    // calc Forces 
    var motorForce1 = inRange(motor.minRPM, motor.maxRPM, drone.hoverRPM - (motor.rpm2force * controlOutput));
    var motorForce2 = inRange(motor.minRPM, motor.maxRPM, drone.hoverRPM + (motor.rpm2force * controlOutput));
    var motorForce = motorForce1 + motorForce2;
    var motorTorque = motorForce2 * drone.armLength - motorForce1 * drone.armLength;

    droneState.alpha = (motorTorque + disturbance.stepAmp) / drone.inertia;

    droneState.omega = droneState.alpha * (timeStep / 1000) + droneState.omega;

    droneState.theta = (((droneState.omega * (timeStep / 1000) + droneState.theta) + 180) % 360) - 180;

    droneState.alpha = motorForce * Math.sin(droneState.theta) / drone.mass;

    droneState.velocity = droneState.alpha * (timeStep / 1000) + droneState.velocity
    // droneState.acceleration =
    // console.log(droneState.theta)

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
var time_stamp = 0;
function animate(image) {
    canvasWidth = csv.width;
    canvasHeight = csv.height;
    physics();
    ctx.clearRect(0, 0, canvasWidth, canvasHeight);
    drawDrone(ctx, image, canvasHeight / 2, canvasWidth / 2, droneState.theta);
    updateDisplay();
    time_stamp = time_stamp + 1;
    graph.push([{ time: time_stamp, y: droneState.theta }]);

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

    if (isNaN(parseFloat(document.getElementById("attDGain").value))) {
        attGains.d = 0;
    } else {
        attGains.d = parseFloat(document.getElementById("attDGain").value);
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

    if (isNaN(parseFloat(document.getElementById("velDGain").value))) {
        velGains.d = 0;
    } else {
        velGains.d = parseFloat(document.getElementById("velDGain").value);
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

    if (isNaN(parseFloat(document.getElementById("posDGain").value))) {
        posGains.d = 0;
    } else {
        posGains.d = parseFloat(document.getElementById("posDGain").value);
    }

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
    // 
    // console.log(disturbance)  
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
    document.getElementById("attDGain").disabled = !enableDisable;
    document.getElementById("attLoopChBx").checked = enableDisable;

    document.getElementById("desiredRate").disabled = enableDisable;
    document.getElementById("desiredAtt").disabled = !enableDisable;
    document.getElementById("desiredVel").disabled = enableDisable;
    document.getElementById("desiredPos").disabled = enableDisable;
}
function enableVel(enableDisable) {
    document.getElementById("velPGain").disabled = !enableDisable;
    document.getElementById("velIGain").disabled = !enableDisable;
    document.getElementById("velDGain").disabled = !enableDisable;
    document.getElementById("velLoopChBx").checked = enableDisable;

    document.getElementById("desiredRate").disabled = enableDisable;
    document.getElementById("desiredAtt").disabled = enableDisable;
    document.getElementById("desiredVel").disabled = !enableDisable;
    document.getElementById("desiredPos").disabled = enableDisable;
}

function enablePos(enableDisable) {
    document.getElementById("posPGain").disabled = !enableDisable;
    document.getElementById("posIGain").disabled = !enableDisable;
    document.getElementById("posDGain").disabled = !enableDisable;
    document.getElementById("posLoopChBx").checked = enableDisable;

    document.getElementById("desiredRate").disabled = enableDisable;
    document.getElementById("desiredAtt").disabled = enableDisable;
    document.getElementById("desiredVel").disabled = enableDisable;
    document.getElementById("desiredPos").disabled = !enableDisable;
}

function updateLoopsEnabled() {
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

function initDefaults() {
    document.getElementById("ratePGain").value = rateGains.p;
    document.getElementById("rateIGain").value = rateGains.i;
    document.getElementById("rateDGain").value = rateGains.d;

    document.getElementById("attPGain").value = attGains.p;
    document.getElementById("attIGain").value = attGains.i;
    document.getElementById("attDGain").value = attGains.d;

    document.getElementById("stepAmp").value = 5000;
    document.getElementById("stepLength").value = 20;

    document.getElementById("desiredRate").value = desired.desiredRate;
    document.getElementById("desiredAtt").value = desired.desiredAtt;
    document.getElementById("desiredVel").value = desired.desiredVel;
    document.getElementById("desiredPos").value = desired.desiredPos;

    enableRate(false);
    enableAtt(false);
    enableVel(false);
    enablePos(false);
    disableDesiredInput();
}





// var trace1 = {
//     x: [1, 2, 3, 4],
//     y: [10, 15, 13, 17],
//     type: 'scatter'
//   };

//   var trace2 = {
//     x: [1, 2, 3, 4],
//     y: [16, 5, 11, 9],
//     type: 'scatter'
//   };

//   var data = [trace1, trace2];

//   Plotly.newPlot('plotDiv', data);

initDefaults();
setInterval(() => {
    animate(droneImage);
}, timeStep)
