import * as THREE from 'three';
import { GUI } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { OrbitControls } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import { DragStateManager } from './utils/DragStateManager.js';
import { Reflector } from './utils/Reflector.js';
import load_mujoco from '../dist/mujoco_wasm.js';

// Load the MuJoCo Module
const mujoco = await load_mujoco();

// Set up Emscripten's Virtual File System
mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');

// Utility functions
function getPosition(buffer, index, target, swizzle = true) {
  if (swizzle) {
    return target.set(
       buffer[(index * 3) + 0],
       buffer[(index * 3) + 2],
      -buffer[(index * 3) + 1]);
  } else {
    return target.set(
       buffer[(index * 3) + 0],
       buffer[(index * 3) + 1],
       buffer[(index * 3) + 2]);
  }
}

function getQuaternion(buffer, index, target, swizzle = true) {
  if (swizzle) {
    return target.set(
      -buffer[(index * 4) + 1],
      -buffer[(index * 4) + 3],
       buffer[(index * 4) + 2],
      -buffer[(index * 4) + 0]);
  } else {
    return target.set(
       buffer[(index * 4) + 0],
       buffer[(index * 4) + 1],
       buffer[(index * 4) + 2],
       buffer[(index * 4) + 3]);
  }
}

function toMujocoPos(target) { 
  return target.set(target.x, -target.z, target.y); 
}

function standardNormal() {
  return Math.sqrt(-2.0 * Math.log(Math.random())) *
         Math.cos(2.0 * Math.PI * Math.random()); 
}

// Load scene from XML file
async function loadSceneFromURL(mujoco, filename, parent) {
  // Load in the state from XML.
  let model, state, simulation;
  try {
    model = mujoco.Model.load_from_xml("/working/" + filename);
    state = new mujoco.State(model);
    simulation = new mujoco.Simulation(model, state);
  } catch (e) {
    console.error("Error loading model from XML:", filename);
    console.error("Exception:", e);
    throw e;
  }

  // Decode the null-terminated string names.
  let textDecoder = new TextDecoder("utf-8");
  let fullString = textDecoder.decode(model.names);
  let names = fullString.split(textDecoder.decode(new ArrayBuffer(1)));

  // Create the root object.
  let mujocoRoot = new THREE.Group();
  mujocoRoot.name = "MuJoCo Root"
  parent.scene.add(mujocoRoot);

  let bodies = {};
  let meshes = {};
  let lights = [];

  // Default material definition.
  let material = new THREE.MeshPhysicalMaterial();
  material.color = new THREE.Color(1, 1, 1);

  // Loop through the MuJoCo geoms and recreate them in three.js.
  for (let g = 0; g < model.ngeom; g++) {
    // Only visualize geom groups up to 2 (same default behavior as simulate).
    if (!(model.geom_group[g] < 3)) { continue; }

    let b = model.geom_bodyid[g];
    let type = model.geom_type[g];
    let size = [
      model.geom_size[(g*3) + 0],
      model.geom_size[(g*3) + 1],
      model.geom_size[(g*3) + 2]
    ];

    // Create the body if it doesn't exist.
    if (!(b in bodies)) {
      bodies[b] = new THREE.Group();
      bodies[b].name = names[model.name_bodyadr[b]];
      bodies[b].bodyID = b;
      bodies[b].has_custom_mesh = false;
    }

    // Set the default geometry. In MuJoCo, this is a sphere.
    let geometry = new THREE.SphereGeometry(size[0] * 0.5);
    if (type == mujoco.mjtGeom.mjGEOM_PLANE.value) {
      // Special handling for plane later.
    } else if (type == mujoco.mjtGeom.mjGEOM_SPHERE.value) {
      geometry = new THREE.SphereGeometry(size[0]);
    } else if (type == mujoco.mjtGeom.mjGEOM_CAPSULE.value) {
      geometry = new THREE.CapsuleGeometry(size[0], size[1] * 2.0, 20, 20);
    } else if (type == mujoco.mjtGeom.mjGEOM_ELLIPSOID.value) {
      geometry = new THREE.SphereGeometry(1);
    } else if (type == mujoco.mjtGeom.mjGEOM_CYLINDER.value) {
      geometry = new THREE.CylinderGeometry(size[0], size[0], size[1] * 2.0);
    } else if (type == mujoco.mjtGeom.mjGEOM_BOX.value) {
      geometry = new THREE.BoxGeometry(size[0] * 2.0, size[2] * 2.0, size[1] * 2.0);
    } else if (type == mujoco.mjtGeom.mjGEOM_MESH.value) {
      let meshID = model.geom_dataid[g];

      if (!(meshID in meshes)) {
        geometry = new THREE.BufferGeometry();

        let vertex_buffer = model.mesh_vert.subarray(
           model.mesh_vertadr[meshID] * 3,
          (model.mesh_vertadr[meshID] + model.mesh_vertnum[meshID]) * 3);
        for (let v = 0; v < vertex_buffer.length; v+=3){
          let temp = vertex_buffer[v + 1];
          vertex_buffer[v + 1] = vertex_buffer[v + 2];
          vertex_buffer[v + 2] = -temp;
        }

        let normal_buffer = model.mesh_normal.subarray(
           model.mesh_vertadr[meshID] * 3,
          (model.mesh_vertadr[meshID] + model.mesh_vertnum[meshID]) * 3);
        for (let v = 0; v < normal_buffer.length; v+=3){
          let temp = normal_buffer[v + 1];
          normal_buffer[v + 1] = normal_buffer[v + 2];
          normal_buffer[v + 2] = -temp;
        }

        let uv_buffer = model.mesh_texcoord.subarray(
           model.mesh_texcoordadr[meshID] * 2,
          (model.mesh_texcoordadr[meshID] + model.mesh_vertnum[meshID]) * 2);
        let triangle_buffer = model.mesh_face.subarray(
           model.mesh_faceadr[meshID] * 3,
          (model.mesh_faceadr[meshID] + model.mesh_facenum[meshID]) * 3);
        geometry.setAttribute("position", new THREE.BufferAttribute(vertex_buffer, 3));
        geometry.setAttribute("normal", new THREE.BufferAttribute(normal_buffer, 3));
        geometry.setAttribute("uv", new THREE.BufferAttribute(uv_buffer, 2));
        geometry.setIndex(Array.from(triangle_buffer));
        meshes[meshID] = geometry;
      } else {
        geometry = meshes[meshID];
      }

      bodies[b].has_custom_mesh = true;
    }

    // Set the Material Properties
    let color = [
      model.geom_rgba[(g * 4) + 0],
      model.geom_rgba[(g * 4) + 1],
      model.geom_rgba[(g * 4) + 2],
      model.geom_rgba[(g * 4) + 3]];
    
    let materialProps = {
      color: new THREE.Color(color[0], color[1], color[2]),
      transparent: color[3] < 1.0,
      opacity: color[3]
    };
    
    if (model.geom_matid[g] != -1) {
      let matId = model.geom_matid[g];
      color = [
        model.mat_rgba[(matId * 4) + 0],
        model.mat_rgba[(matId * 4) + 1],
        model.mat_rgba[(matId * 4) + 2],
        model.mat_rgba[(matId * 4) + 3]];
      materialProps.color = new THREE.Color(color[0], color[1], color[2]);
      materialProps.transparent = color[3] < 1.0;
      materialProps.opacity = color[3];
      materialProps.specularIntensity = model.mat_specular[matId] * 0.5;
      materialProps.reflectivity = model.mat_reflectance[matId];
      materialProps.roughness = 1.0 - model.mat_shininess[matId];
      materialProps.metalness = 0.1;
    }
    
    material = new THREE.MeshPhysicalMaterial(materialProps);

    let mesh = new THREE.Mesh();
    if (type == 0) {
      mesh = new Reflector(new THREE.PlaneGeometry(100, 100), { clipBias: 0.003 });
      mesh.rotateX(-Math.PI / 2);
    } else {
      mesh = new THREE.Mesh(geometry, material);
    }

    mesh.castShadow = g == 0 ? false : true;
    mesh.receiveShadow = type != 7;
    mesh.bodyID = b;
    bodies[b].add(mesh);
    getPosition(model.geom_pos, g, mesh.position);
    if (type != 0) { getQuaternion(model.geom_quat, g, mesh.quaternion); }
    if (type == 4) { mesh.scale.set(size[0], size[2], size[1]) }
  }

  // Parse lights.
  for (let l = 0; l < model.nlight; l++) {
    let light = new THREE.SpotLight();
    if (model.light_directional[l]) {
      light = new THREE.DirectionalLight(0xffffff, 1.0);
      light.position.set(2, 4, 3);
    } else {
      light = new THREE.SpotLight(0xffffff, 1.0);
    }
    light.decay = model.light_attenuation[l] * 100;
    light.penumbra = 0.5;
    light.castShadow = true;
    light.shadow.mapSize.width = 2048;
    light.shadow.mapSize.height = 2048;
    light.shadow.camera.near = 0.1;
    light.shadow.camera.far = 20;
    light.shadow.bias = -0.0001;
    
    if (bodies[0]) {
      bodies[0].add(light);
    } else {
      mujocoRoot.add(light);
    }
    lights.push(light);
  }
  if (model.nlight == 0) {
    let light = new THREE.DirectionalLight(0xffffff, 1.0);
    light.position.set(2, 4, 3);
    light.castShadow = true;
    light.shadow.mapSize.width = 2048;
    light.shadow.mapSize.height = 2048;
    light.shadow.camera.near = 0.1;
    light.shadow.camera.far = 20;
    light.shadow.bias = -0.0001;
    mujocoRoot.add(light);
  }

  // Add bodies to scene
  for (let b = 0; b < model.nbody; b++) {
    // Create body if it doesn't exist
    if (!bodies[b]) {
      bodies[b] = new THREE.Group();
      bodies[b].name = names[model.name_bodyadr[b]] || `body_${b}`;
      bodies[b].bodyID = b;
      bodies[b].has_custom_mesh = false;
    }
    
    // Add to scene hierarchy
    if (b == 0 || !bodies[0]) {
      mujocoRoot.add(bodies[b]);
    } else {
      bodies[0].add(bodies[b]);
    }
  }

  // Add bodies to scene
  for (let b = 0; b < model.nbody; b++) {
    if (b == 0 || !bodies[0]) {
      mujocoRoot.add(bodies[b]);
    } else if(bodies[b]){
      bodies[0].add(bodies[b]);
    } else {
      console.log("Body without Geometry detected; adding to bodies", b, bodies[b]);
      bodies[b] = new THREE.Group(); 
      bodies[b].name = names[b + 1]; 
      bodies[b].bodyID = b; 
      bodies[b].has_custom_mesh = false;
      bodies[0].add(bodies[b]);
    }
  }

  parent.mujocoRoot = mujocoRoot;

  return [model, state, simulation, bodies, lights];
}

// Download Franka FR3 scene files
async function downloadFrankaScene(mujoco) {
  let allFiles = [
    "franka_fr3/scene.xml",
    "franka_fr3/fr3.xml",
    "franka_fr3/assets/link0.obj",
    "franka_fr3/assets/link0.stl",
    "franka_fr3/assets/link0_0.obj",
    "franka_fr3/assets/link0_1.obj",
    "franka_fr3/assets/link0_2.obj",
    "franka_fr3/assets/link0_3.obj",
    "franka_fr3/assets/link0_4.obj",
    "franka_fr3/assets/link0_5.obj",
    "franka_fr3/assets/link0_6.obj",
    "franka_fr3/assets/link1.obj",
    "franka_fr3/assets/link1.stl",
    "franka_fr3/assets/link2.obj",
    "franka_fr3/assets/link2.stl",
    "franka_fr3/assets/link3.obj",
    "franka_fr3/assets/link3.stl",
    "franka_fr3/assets/link3_0.obj",
    "franka_fr3/assets/link3_1.obj",
    "franka_fr3/assets/link4.obj",
    "franka_fr3/assets/link4.stl",
    "franka_fr3/assets/link4_0.obj",
    "franka_fr3/assets/link4_1.obj",
    "franka_fr3/assets/link5.obj",
    "franka_fr3/assets/link5.stl",
    "franka_fr3/assets/link5_0.obj",
    "franka_fr3/assets/link5_1.obj",
    "franka_fr3/assets/link5_2.obj",
    "franka_fr3/assets/link6.obj",
    "franka_fr3/assets/link6.stl",
    "franka_fr3/assets/link6_0.obj",
    "franka_fr3/assets/link6_1.obj",
    "franka_fr3/assets/link6_2.obj",
    "franka_fr3/assets/link6_3.obj",
    "franka_fr3/assets/link6_4.obj",
    "franka_fr3/assets/link6_5.obj",
    "franka_fr3/assets/link6_6.obj",
    "franka_fr3/assets/link6_7.obj",
    "franka_fr3/assets/link7.obj",
    "franka_fr3/assets/link7.stl",
    "franka_fr3/assets/link7_0.obj",
    "franka_fr3/assets/link7_1.obj",
    "franka_fr3/assets/link7_2.obj",
    "franka_fr3/assets/link7_3.obj",
  ];

  let requests = allFiles.map((url) => fetch("./examples/scenes/" + url));
  let responses = await Promise.all(requests);
  
  for (let i = 0; i < responses.length; i++) {
    let split = allFiles[i].split("/");
    let working = '/working/';
    for (let f = 0; f < split.length - 1; f++) {
      working += split[f];
      if (!mujoco.FS.analyzePath(working).exists) { 
        mujoco.FS.mkdir(working); 
      }
      working += "/";
    }

    if (allFiles[i].endsWith(".obj") || allFiles[i].endsWith(".png") || allFiles[i].endsWith(".stl")) {
      mujoco.FS.writeFile("/working/" + allFiles[i], new Uint8Array(await responses[i].arrayBuffer()));
    } else {
      mujoco.FS.writeFile("/working/" + allFiles[i], await responses[i].text());
    }
  }
}

export class FrankaDemo {
  constructor() {
    this.mujoco = mujoco;

    // Model will be loaded in init()
    this.model = null;
    this.state = null;
    this.simulation = null;

    // Define state variables
    this.params = { 
      paused: false, 
      ctrlnoiserate: 0.0, 
      ctrlnoisestd: 0.0,
      cameraFOV: 75,
      cameraDistance: 0.8
    };
    this.mujoco_time = 0.0;
    this.bodies = {};
    this.lights = {};
    this.tmpVec = new THREE.Vector3();
    this.tmpQuat = new THREE.Quaternion();
    this.updateGUICallbacks = [];

    // Main view container
    this.container = document.createElement('div');
    document.body.appendChild(this.container);

    // Main scene
    this.scene = new THREE.Scene();
    this.scene.name = 'scene';

    // Main camera
    this.camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.001, 100);
    this.camera.name = 'PerspectiveCamera';
    this.camera.position.set(1.0, 0.8, 1.0);
    this.scene.add(this.camera);

    this.scene.background = new THREE.Color(0.15, 0.25, 0.35);
    this.scene.fog = new THREE.Fog(this.scene.background, 15, 25.5);

    this.ambientLight = new THREE.AmbientLight(0xffffff, 0.3);
    this.ambientLight.name = 'AmbientLight';
    this.scene.add(this.ambientLight);

    // Main renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;  // Better color accuracy
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;  // More realistic tone mapping
    this.renderer.toneMappingExposure = 1.0;
    this.renderer.setAnimationLoop(this.render.bind(this));

    this.container.appendChild(this.renderer.domElement);

    // Orbit controls for main view
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.3, 0);
    this.controls.panSpeed = 2;
    this.controls.zoomSpeed = 1;
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.10;
    this.controls.screenSpacePanning = true;
    this.controls.update();

    // Virtual camera for rendering camera view
    this.virtualCamera = new THREE.PerspectiveCamera(75, 320 / 240, 0.01, 10);
    this.virtualCamera.name = 'VirtualCamera';
    this.scene.add(this.virtualCamera);

    // Camera view canvas and renderer
    this.cameraCanvas = document.getElementById('camera-view');
    this.cameraCanvas.width = 320;
    this.cameraCanvas.height = 240;
    
    this.cameraRenderer = new THREE.WebGLRenderer({ 
      canvas: this.cameraCanvas, 
      antialias: true,
      alpha: false
    });
    this.cameraRenderer.setSize(320, 240);
    this.cameraRenderer.shadowMap.enabled = true;
    this.cameraRenderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.cameraRenderer.outputColorSpace = THREE.SRGBColorSpace;
    this.cameraRenderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.cameraRenderer.toneMappingExposure = 1.0;

    window.addEventListener('resize', this.onWindowResize.bind(this));

    // Initialize the Drag State Manager
    this.dragStateManager = new DragStateManager(
      this.scene, 
      this.renderer, 
      this.camera, 
      this.container.parentElement, 
      this.controls
    );
  }

  async init() {
    // Download the Franka FR3 scene files
    await downloadFrankaScene(mujoco);

    // Initialize the three.js Scene using the Franka scene
    [this.model, this.state, this.simulation, this.bodies, this.lights] =
      await loadSceneFromURL(mujoco, "franka_fr3/scene.xml", this);

    // Position the virtual camera (mounted on the end effector would be ideal)
    // For now, we'll position it to look at the robot from a fixed position
    this.updateVirtualCamera();

    this.setupGUI();
  }

  setupGUI() {
    this.gui = new GUI();

    let simulationFolder = this.gui.addFolder("Simulation");

    // Add pause simulation checkbox
    const pauseSimulation = simulationFolder.add(this.params, 'paused').name('Pause Simulation');
    pauseSimulation.onChange((value) => {
      if (value) {
        const pausedText = document.createElement('div');
        pausedText.style.position = 'absolute';
        pausedText.style.top = '10px';
        pausedText.style.right = '10px';
        pausedText.style.color = 'white';
        pausedText.style.font = 'normal 18px Arial';
        pausedText.style.backgroundColor = 'rgba(0, 0, 0, 0.5)';
        pausedText.style.padding = '5px 10px';
        pausedText.style.borderRadius = '3px';
        pausedText.innerHTML = 'PAUSED';
        pausedText.id = 'paused-indicator';
        this.container.appendChild(pausedText);
      } else {
        const indicator = document.getElementById('paused-indicator');
        if (indicator) indicator.remove();
      }
    });

    document.addEventListener('keydown', (event) => {
      if (event.code === 'Space') {
        this.params.paused = !this.params.paused;
        pauseSimulation.setValue(this.params.paused);
        event.preventDefault();
      }
    });

    // Add reset simulation button
    const resetSimulation = () => {
      this.simulation.resetData();
      this.simulation.forward();
    };
    simulationFolder.add({ reset: resetSimulation }, 'reset').name('Reset');
    
    document.addEventListener('keydown', (event) => {
      if (event.code === 'Backspace') {
        resetSimulation();
        event.preventDefault();
      }
    });

    // Add noise controls
    simulationFolder.add(this.params, 'ctrlnoiserate', 0.0, 2.0, 0.01).name('Noise rate');
    simulationFolder.add(this.params, 'ctrlnoisestd', 0.0, 2.0, 0.01).name('Noise scale');

    // Camera settings
    let cameraFolder = this.gui.addFolder("Virtual Camera");
    cameraFolder.add(this.params, 'cameraFOV', 30, 120, 1).name('FOV').onChange((value) => {
      this.virtualCamera.fov = value;
      this.virtualCamera.updateProjectionMatrix();
    });
    cameraFolder.add(this.params, 'cameraDistance', 0.1, 2.0, 0.05).name('Distance');

    // Actuator controls
    let actuatorFolder = simulationFolder.addFolder("Actuators");
    let textDecoder = new TextDecoder("utf-8");
    let nullChar = textDecoder.decode(new ArrayBuffer(1));
    
    for (let i = 0; i < this.model.nu; i++) {
      if (!this.model.actuator_ctrllimited[i]) { continue; }
      
      let name = textDecoder.decode(
        this.model.names.subarray(this.model.name_actuatoradr[i])
      ).split(nullChar)[0];

      let act_range = this.model.actuator_ctrlrange;
      this.params[name] = 0.0;
      
      let actuatorGUI = actuatorFolder.add(
        this.params, 
        name, 
        act_range[2 * i], 
        act_range[2 * i + 1], 
        0.01
      ).name(name).listen();
      
      actuatorGUI.onChange((value) => {
        this.simulation.ctrl[i] = value;
      });
    }
    actuatorFolder.close();

    simulationFolder.open();
    this.gui.open();
  }

  updateVirtualCamera() {
    // Position the camera to look at the robot from above and in front
    // You can modify this to attach the camera to a specific body/link
    const distance = this.params.cameraDistance;
    const height = 0.6;
    const lookAtHeight = 0.3;
    
    // Position camera in front of and slightly above the robot
    this.virtualCamera.position.set(distance * 0.7, height, distance * 0.7);
    this.virtualCamera.lookAt(0, lookAtHeight, 0);
    this.virtualCamera.updateMatrixWorld();
  }

  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }

  render(timeMS) {
    this.controls.update();

    // Check if model and simulation are loaded and valid
    if (!this.model || !this.simulation) {
      this.renderer.render(this.scene, this.camera);
      return;
    }

    if (!this.params["paused"]) {
      let timestep = this.model.getOptions().timestep;
      if (timeMS - this.mujoco_time > 35.0) {
        this.mujoco_time = timeMS;
      }
      
      while (this.mujoco_time < timeMS) {
        // Jitter the control state with gaussian random noise
        if (this.params["ctrlnoisestd"] > 0.0) {
          let rate = Math.exp(-timestep / Math.max(1e-10, this.params["ctrlnoiserate"]));
          let scale = this.params["ctrlnoisestd"] * Math.sqrt(1 - rate * rate);
          let currentCtrl = this.simulation.ctrl;
          for (let i = 0; i < currentCtrl.length; i++) {
            currentCtrl[i] = rate * currentCtrl[i] + scale * standardNormal();
            let textDecoder = new TextDecoder("utf-8");
            let nullChar = textDecoder.decode(new ArrayBuffer(1));
            let name = textDecoder.decode(
              this.model.names.subarray(this.model.name_actuatoradr[i])
            ).split(nullChar)[0];
            this.params[name] = currentCtrl[i];
          }
        }

        // Clear old perturbations, apply new ones
        for (let i = 0; i < this.simulation.qfrc_applied.length; i++) {
          this.simulation.qfrc_applied[i] = 0.0;
        }
        
        let dragged = this.dragStateManager.physicsObject;
        if (dragged && dragged.bodyID) {
          for (let b = 0; b < this.model.nbody; b++) {
            if (this.bodies[b]) {
              getPosition(this.simulation.xpos, b, this.bodies[b].position);
              getQuaternion(this.simulation.xquat, b, this.bodies[b].quaternion);
              this.bodies[b].updateWorldMatrix();
            }
          }
          let bodyID = dragged.bodyID;
          this.dragStateManager.update();
          let force = toMujocoPos(
            this.dragStateManager.currentWorld.clone()
              .sub(this.dragStateManager.worldHit)
              .multiplyScalar(this.model.body_mass[bodyID] * 250)
          );
          let point = toMujocoPos(this.dragStateManager.worldHit.clone());
          this.simulation.applyForce(
            force.x, force.y, force.z, 
            0, 0, 0, 
            point.x, point.y, point.z, 
            bodyID
          );
        }

        this.simulation.step();
        this.mujoco_time += timestep * 1000.0;
      }
    } else if (this.params["paused"]) {
      this.dragStateManager.update();
      let dragged = this.dragStateManager.physicsObject;
      if (dragged && dragged.bodyID) {
        let b = dragged.bodyID;
        let offset = toMujocoPos(
          this.dragStateManager.currentWorld.clone()
            .sub(this.dragStateManager.worldHit)
            .multiplyScalar(0.3)
        );
        
        if (this.model.body_mocapid[b] >= 0) {
          let addr = this.model.body_mocapid[b] * 3;
          let pos = this.simulation.mocap_pos;
          pos[addr + 0] += offset.x;
          pos[addr + 1] += offset.y;
          pos[addr + 2] += offset.z;
        } else {
          let root = this.model.body_rootid[b];
          let addr = this.model.jnt_qposadr[this.model.body_jntadr[root]];
          let pos = this.simulation.qpos;
          pos[addr + 0] += offset.x;
          pos[addr + 1] += offset.y;
          pos[addr + 2] += offset.z;
        }
      }
      this.simulation.forward();
    }

    // Update body transforms
    for (let b = 0; b < this.model.nbody; b++) {
      if (this.bodies[b]) {
        getPosition(this.simulation.xpos, b, this.bodies[b].position);
        getQuaternion(this.simulation.xquat, b, this.bodies[b].quaternion);
        this.bodies[b].updateWorldMatrix();
      }
    }

    // Update light transforms
    for (let l = 0; l < this.model.nlight; l++) {
      if (this.lights[l]) {
        getPosition(this.simulation.light_xpos, l, this.lights[l].position);
        getPosition(this.simulation.light_xdir, l, this.tmpVec);
        this.lights[l].lookAt(this.tmpVec.add(this.lights[l].position));
      }
    }

    // Update virtual camera position
    this.updateVirtualCamera();

    // Render main view
    this.renderer.render(this.scene, this.camera);

    // Render camera view
    this.cameraRenderer.render(this.scene, this.virtualCamera);
  }
}

let demo = new FrankaDemo();
await demo.init();
