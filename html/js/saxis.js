// saxis.js is all of the logic for the saxis robot rendering.

const margin = 15;

var camera;
var scene;
var renderer;
var viewer;
var onAir = false; // true when recording video
var recording;
var material;
var matBlack;

var saxis;
var axes;

var eStop = false;
var eHold = true;

var ang = function(a) { return Math.PI * a / 180.0; };
var deg = function(r) { return 180.0 * r / Math.PI; };

function dID(x) {
  return document.getElementById(x);
}

function onWindowResize() {
  viewer.width = window.innerWidth * .67;
  viewer.height = window.innerHeight;
  camera.aspect = viewer.width / viewer.height;
  camera.updateProjectionMatrix();
  renderer.setSize( viewer.width, viewer.height );
}

function rpc(input, callbackFn) {
  var x = new XMLHttpRequest();
  x.open('POST', 'rpc', true);
  x.setRequestHeader('Content-type', 'application/x-www-form-urlencoded');
  x.onreadystatechange = function() {
    if (x.readyState != 4) {
      return;
    }
    if (x.status != 200) {
      callbackFn(input, {Error: "Server and client connection lost!"});
      eStop = true;
      return;
    }
    var raw;
    try {
      raw = JSON.parse(x.responseText);
    } catch (ex) {
      callbackFn(input, {
        Error: 'bad JSON (' + ex + '):' + x.responseText
      });
      return;
    }
    if (raw.Error) {
      callbackFn(input, {Error: raw.Error});
      return;
    }
    callbackFn(input, raw);
  };
  x.send('rpc=' + JSON.stringify(input));
}

function addCurve(a, b) {
  var mat = new THREE.LineBasicMaterial( { color: 0x0000ff } );
  var pts = [];
  for (var i = 0; i < b.length; i++) {
    var v = b[i];
    pts.push(new THREE.Vector3(v[0], v[1], v[2]));
  }
  var g = new THREE.BufferGeometry().setFromPoints(pts);
  var line = new THREE.Line(g, mat);

  scene.add(line);
}

// basePose is used to set the pose at the end of the most recently
// executed pace.
function basePose(js) {
  for (var i = 0; i < js.length; i++) {
    saxis.baseJS[i] = js[i];
  }
}

function adoptPose(js) {
  for (var i = 0; i < js.length; i++) {
    switch (saxis.Robot[i].Axis) {
    case 'x':
      axes[i].rotation.x = js[i];
      break;
    case 'y':
      axes[i].rotation.y = js[i];
      break;
    case 'z':
      axes[i].rotation.z = js[i];
      break;
    }
    let el = dID(`J${i}ang`);
    if(el) {
      el.value = deg(js[i]).toFixed(2);
    }
  }
}

var statusInFlight = false;
var recordedData = [];

function appendRecordedData(ev) {
  recordedData.push(ev.data);
}

function addStatus(a, b) {
  statusInFlight = false;
  if (b == null) {
    return;
  }

  if (b.Program && !saxis.program) {
    if (saxis.pcount == null || saxis.pcount < b.Pcount) {
      console.log("loading program ", b.Pcount);
      saxis.pcount = b.Pcount;
      dID('progNo').innerHTML = saxis.pcount;
      saxis.pc = 0;  // start of the program.
      saxis.program = b.Program;
      saxis.done = false;
      eHold = false;
      saxis.clockStarted = Date.now();
      if (b.Pcount == 4) {
        var stream = viewer.captureStream(24);
        recording = new MediaRecorder(stream, {mimeType: 'video/webm'});
        recording.ondataavailable = appendRecordedData;
        onAir = true;
        var airing = dID('airing');
        airing.innerHTML = 'ON AIR';
        airing.style.backgroundColor = '#e00';
        recording.start(100);
        console.log('started recording');
      } else if (b.Pcount == 5) {
        recording.stop();
        console.log('stopped recording');
        onAir = false;
        var airing = dID('airing');
        airing.innerHTML = 'SAVE VIDEO';
        airing.style.backgroundColor = '#0c0';
        airing.onclick = saveRecording;
      }
    }
  }

  if (b.Pose) {
    if (saxis.pcount == null) {
      saxis.Pose = b.Pose;
      console.log("adopting pose");
      adoptPose(b.Pose.J);
      basePose(b.Pose.J);
    }
  }
}

function scheduleStatus() {
  if (saxis && !statusInFlight) {
    statusInFlight = true;
    rpc({
      Cmd: 'status',
      Pcount: saxis.done ? saxis.pcount : 0,
    }, addStatus);
  }
  if (!eStop) {
    setTimeout(scheduleStatus, 213);
  }
}

function addRobot(a, b) {
  scheduleStatus();

  material = new THREE.MeshPhongMaterial({
    color: 0xFF00FF,
    specular: 0xFF00FF,
    emissive: 0xAA00EE,
    shininess: 100,
    flatShading: true
  });

  var matBlack = new THREE.MeshPhongMaterial({
    color: 0x111111,
    specular: 0xFFFFFF,
    emissive: 0x111111,
    shininess: 100,
    flatShading: true
  });

  saxis = b;
  axes = [];
  for (var i = 0; i<saxis.Robot.length; i++) {
    var j = saxis.Robot[i];
    var c = null, g;
    switch (j.Axis) {
    case 'z':
      var hw = j.Width/2;
      g = new THREE.CylinderGeometry(hw, hw, j.Length+.001, 8);
      g.rotateX(ang(90));
      g.translate(0, 0, j.Length/2);
      break;
    default:
      c = new THREE.CylinderGeometry(hw/2, hw/2, saxis.Robot[i-1].Width, 8);
      c.rotateZ(ang(90));
      g = new THREE.BoxGeometry(j.Width, j.Width, j.Length);
      g.translate(0, 0, j.Length/2);
    }
    var axis = new THREE.Mesh(g, i == 5 ? matBlack : material);
    if (c) {
      var hinge = new THREE.Mesh(c, material);
      axis.add(hinge);
    }
    if (i != 0) {
      axis.position.z = saxis.Robot[i-1].Length;
      axes[i-1].add(axis);
    }
    axes.push(axis);
  }
  adoptPose(saxis.Pose.J);
  saxis.baseJS = new Array(saxis.Pose.J.length);
  basePose(saxis.Pose.J);

  scene.add(axes[0]);
}

function programDone() {
  saxis.pc = 0;
  saxis.program = null;
  saxis.done = true;
  eHold = true;
}

// TODO execute a motion script pc.
function doMotion() {
  if (eHold) {
    return;
  }

  if (saxis.pc >= saxis.program.length) {
    programDone();
    return;
  }

  var pacer = saxis.program[saxis.pc];
  if (!pacer) {
    return;
  }

  var ts = (Date.now() - saxis.clockStarted) * 0.001;

  var last = pacer[pacer.length - 1];
  while (last.Frac <= ts) {
    // already past the end of this pacer.
    basePose(last.J);
    adoptPose(last.J);

    saxis.pc += 1;
    saxis.clockStarted += 1000*last.Frac;
    ts -= last.Frac;

    pacer = saxis.program[saxis.pc];
    if (!pacer) {
      programDone();
      return;
    }

    last = pacer[pacer.length - 1];
  }

  var dt = 0.0;
  var next;
  for (var i = 0; i<pacer.length; i++) {
    next = pacer[i];
    if (next.Frac > ts) {
      break;
    }
    dt = next.Frac;
    basePose(next.J);
  }

  // diff indicates how far between last and next the joints are.
  var diff = (ts-dt)/(next.Frac-dt);

  var oneLessDiff = 1.0 - diff;
  var js = saxis.Pose.J;
  for (var j = 0; j < next.J.length; j++) {
    js[j] = saxis.baseJS[j] * oneLessDiff + diff * next.J[j];
  }

  // Update pose.
  adoptPose(js);
}

function saxis_init(view) {
  THREE.Object3D.DefaultUp = new THREE.Vector3(0,0,1);
  scene = new THREE.Scene();

  // threejs default coordinates are Z towards us, X to the right and
  // Y upwards. Saxis uses a different coordinate orientation, so we
  // adjust the camera to support that. Saxis uses, X to the right, Y
  // into the screen (away from the viewer) and Z upwards.
  camera = new THREE.PerspectiveCamera(75, window.innerWidth/window.innerHeight);
  camera.position.y = -6.8;

  // Add a little slant to get a bit more perspective.
  camera.position.z = 1;
  camera.position.x = 1.2;
  camera.rotation.x += ang(90);
  camera.rotation.y += ang(20);

  viewer = dID(view);
  renderer = new THREE.WebGLRenderer({
    antialias: true,
    canvas: viewer
  });
  renderer.setClearColor(0xFFFFFF, 1);
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(window.innerWidth-margin, window.innerHeight-margin);

  onWindowResize();

  pointLight = new THREE.DirectionalLight(0xffffff, 1);
  pointLight.position.x = 0;
  pointLight.position.y = 0;
  pointLight.position.z = -2;
  scene.add(pointLight);

  var animate = function() {
    if (!eStop) {
      if (saxis) {
        doMotion();
      }
    }
    renderer.render(scene, camera);
    requestAnimationFrame(animate);
  };
  animate();

  rpc({Cmd: 'hilbert'}, addCurve);
  rpc({Cmd: 'scene'}, addRobot);

  window.addEventListener('resize', onWindowResize, false);
}

function saveRecording() {
  if (onAir) {
    return;
  }
  var blob = new Blob(recordedData, {type: 'video/webm'});
  var u = window.URL.createObjectURL(blob);
  var a = document.createElement('a');
  a.style.display = 'none';
  a.href = u;
  a.download = 'sample.webm';
  a.click();
  setTimeout(function() {
    window.URL.revokeObjectURL(u);
  }, 100);
}
