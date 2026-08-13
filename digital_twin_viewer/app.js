(function () {
  const canvas = document.getElementById("scene");
  const sourceLabel = document.getElementById("sourceLabel");
  const statusReadout = document.getElementById("statusReadout");
  const tReadout = document.getElementById("tReadout");
  const posReadout = document.getElementById("posReadout");
  const motionReadout = document.getElementById("motionReadout");
  const forceReadout = document.getElementById("forceReadout");
  const windReadout = document.getElementById("windReadout");
  const breakdownReadout = document.getElementById("breakdownReadout");
  const comparisonReadout = document.getElementById("comparisonReadout");
  const comparisonLegend = document.getElementById("comparisonLegend");
  const comparisonLegendItems = document.getElementById("comparisonLegendItems");
  const simPlayPause = document.getElementById("simPlayPause");
  const simReset = document.getElementById("simReset");
  const raceTrack = document.getElementById("raceTrack");
  const cameraModeInput = document.getElementById("cameraMode");
  const terrainType = document.getElementById("terrainType");
  const loadRaceTrackButton = document.getElementById("loadRaceTrack");
  const trackReadout = document.getElementById("trackReadout");
  const terrainReadout = document.getElementById("terrainReadout");
  const compareModel1 = document.getElementById("compareModel1");
  const compareModel2 = document.getElementById("compareModel2");
  const compareModels = document.getElementById("compareModels");
  const compareTerrain1 = document.getElementById("compareTerrain1");
  const compareTerrain2 = document.getElementById("compareTerrain2");
  const compareTerrains = document.getElementById("compareTerrains");
  const modelPreset = document.getElementById("modelPreset");
  const windProfile = document.getElementById("windProfile");
  const muInput = document.getElementById("mu");
  const parameterInputs = [...document.querySelectorAll("[data-parameter]")];
  const forceInputs = [...document.querySelectorAll("[data-force]")];
  let socket = null;
  let latestSnapshot = null;
  let car = null;
  let trailLine = null;
  let comparisonGroup = new THREE.Group();
  let raceTrackGroup = new THREE.Group();
  let trackCatalog = {};
  let activeRaceTrack = null;
  let cameraMode = "chase";
  let comparisonView = false;
  let cameraLookTarget = new THREE.Vector3();
  let tracksideAnchor = null;
  let trackViewCenter = new THREE.Vector3();
  let trackViewSize = new THREE.Vector2(10, 10);
  let lastPathControllerTime = -Infinity;
  let pathFollowerIndex = null;
  let selectedModel = "race";
  let loadedModelId = null;
  let comparisonPlayback = null;
  let activeTerrainId = "asphalt";
  let activeWindProfileId = "calm";
  let activeWindSpeed = 0.0;

  const TERRAIN_VISUALS = {
    asphalt: { label: "Dry asphalt", road: 0x293238, ground: 0x151b1e, roughness: 0.92, pattern: "asphalt" },
    "wet-asphalt": { label: "Wet asphalt", road: 0x1b2d38, ground: 0x101a22, roughness: 0.55, pattern: "wet" },
    gravel: { label: "Gravel", road: 0x726d61, ground: 0x4b4a42, roughness: 1.0, pattern: "gravel" },
    grass: { label: "Grass", road: 0x52694d, ground: 0x30472f, roughness: 1.0, pattern: "grass" },
    mud: { label: "Mud", road: 0x5a4636, ground: 0x392b25, roughness: 1.0, pattern: "mud" },
    ice: { label: "Ice", road: 0x8ba8b4, ground: 0x526b78, roughness: 0.42, pattern: "ice" },
  };

  const WIND_PROFILES = {
    calm: { speed: 0.0, direction_deg: 0.0, aero_drag: 0.32, aero_side_drag: 1.0, air_density: 1.225 },
    light: { speed: 3.0, direction_deg: 90.0, aero_drag: 0.32, aero_side_drag: 1.0, air_density: 1.225 },
    moderate: { speed: 7.0, direction_deg: 90.0, aero_drag: 0.32, aero_side_drag: 1.0, air_density: 1.225 },
    strong: { speed: 13.0, direction_deg: 90.0, aero_drag: 0.32, aero_side_drag: 1.0, air_density: 1.225 },
    storm: { speed: 22.0, direction_deg: 45.0, aero_drag: 0.38, aero_side_drag: 1.15, air_density: 1.225 },
    hurricane: { speed: 32.0, direction_deg: 45.0, aero_drag: 0.42, aero_side_drag: 1.25, air_density: 1.225 },
  };

  // Imported meshes are presentation assets only.  The physics profile below
  // remains the source of truth for mass, dimensions, tire forces, and the
  // six-DOF vehicle state.  Keeping the two separate lets a mesh be replaced
  // without changing the simulated vehicle unexpectedly.
  const MODEL_ASSETS = {
    "cc0-sports-a": {
      url: "./assets/imported_cars/sports-car-a.glb",
      label: "Imported Sports Car A · CC0",
      axis: "x-width-z-forward",
    },
    "cc0-sports-b": {
      url: "./assets/imported_cars/sports-car-b.glb",
      label: "Imported Sports Car B · CC0",
      axis: "x-width-z-forward",
    },
    "cc0-suv": {
      url: "./assets/imported_cars/performance-suv.glb",
      label: "Imported Performance SUV · CC0",
      axis: "x-width-z-forward",
    },
  };

  const MODEL_PROFILES = {
    race: { mass: 3.50, wheelbase: 0.324, yaw_inertia: 0.055, track_width: 0.22, body_length: 0.62, body_width: 0.28, body_height: 0.14, wheel_radius: 0.055, com_height: 0.09, front_cornering_stiffness: 24, rear_cornering_stiffness: 26, tire_longitudinal_stiffness: 30, suspension_stiffness: 120, suspension_damping: 3, roll_inertia: 0.07, pitch_inertia: 0.08, max_drive_torque: 1.2, v_max: 0.8 },
    "race-future": { mass: 3.80, wheelbase: 0.35, yaw_inertia: 0.060, track_width: 0.23, body_length: 0.68, body_width: 0.29, body_height: 0.13, wheel_radius: 0.055, com_height: 0.085, front_cornering_stiffness: 28, rear_cornering_stiffness: 30, tire_longitudinal_stiffness: 34, suspension_stiffness: 140, suspension_damping: 3.5, roll_inertia: 0.075, pitch_inertia: 0.085, max_drive_torque: 1.3, v_max: 0.9 },
    "hatchback-sports": { mass: 4.60, wheelbase: 0.42, yaw_inertia: 0.09, track_width: 0.27, body_length: 0.82, body_width: 0.36, body_height: 0.20, wheel_radius: 0.07, com_height: 0.12, front_cornering_stiffness: 34, rear_cornering_stiffness: 36, tire_longitudinal_stiffness: 38, suspension_stiffness: 170, suspension_damping: 4, roll_inertia: 0.11, pitch_inertia: 0.12, max_drive_torque: 1.6, v_max: 0.9 },
    "sedan-sports": { mass: 5.20, wheelbase: 0.48, yaw_inertia: 0.13, track_width: 0.29, body_length: 0.94, body_width: 0.39, body_height: 0.22, wheel_radius: 0.075, com_height: 0.13, front_cornering_stiffness: 38, rear_cornering_stiffness: 40, tire_longitudinal_stiffness: 42, suspension_stiffness: 190, suspension_damping: 4.5, roll_inertia: 0.16, pitch_inertia: 0.17, max_drive_torque: 1.8, v_max: 1.0 },
    sedan: { mass: 5.80, wheelbase: 0.50, yaw_inertia: 0.15, track_width: 0.30, body_length: 0.98, body_width: 0.40, body_height: 0.24, wheel_radius: 0.08, com_height: 0.14, front_cornering_stiffness: 32, rear_cornering_stiffness: 34, tire_longitudinal_stiffness: 38, suspension_stiffness: 180, suspension_damping: 4, roll_inertia: 0.18, pitch_inertia: 0.19, max_drive_torque: 1.8, v_max: 0.85 },
    suv: { mass: 7.20, wheelbase: 0.52, yaw_inertia: 0.22, track_width: 0.32, body_length: 1.02, body_width: 0.44, body_height: 0.30, wheel_radius: 0.095, com_height: 0.18, front_cornering_stiffness: 28, rear_cornering_stiffness: 30, tire_longitudinal_stiffness: 36, suspension_stiffness: 220, suspension_damping: 6, roll_inertia: 0.27, pitch_inertia: 0.25, max_drive_torque: 2.1, v_max: 0.75 },
    "suv-luxury": { mass: 8.00, wheelbase: 0.54, yaw_inertia: 0.25, track_width: 0.33, body_length: 1.08, body_width: 0.46, body_height: 0.31, wheel_radius: 0.10, com_height: 0.19, front_cornering_stiffness: 31, rear_cornering_stiffness: 33, tire_longitudinal_stiffness: 40, suspension_stiffness: 240, suspension_damping: 7, roll_inertia: 0.30, pitch_inertia: 0.28, max_drive_torque: 2.3, v_max: 0.75 },
    taxi: { mass: 6.20, wheelbase: 0.51, yaw_inertia: 0.16, track_width: 0.30, body_length: 1.00, body_width: 0.41, body_height: 0.25, wheel_radius: 0.08, com_height: 0.145, front_cornering_stiffness: 30, rear_cornering_stiffness: 32, tire_longitudinal_stiffness: 36, suspension_stiffness: 190, suspension_damping: 5, roll_inertia: 0.19, pitch_inertia: 0.20, max_drive_torque: 1.8, v_max: 0.8 },
    van: { mass: 7.50, wheelbase: 0.55, yaw_inertia: 0.24, track_width: 0.33, body_length: 1.10, body_width: 0.46, body_height: 0.32, wheel_radius: 0.10, com_height: 0.19, front_cornering_stiffness: 25, rear_cornering_stiffness: 27, tire_longitudinal_stiffness: 34, suspension_stiffness: 230, suspension_damping: 6, roll_inertia: 0.29, pitch_inertia: 0.28, max_drive_torque: 2.0, v_max: 0.7 },
    delivery: { mass: 8.00, wheelbase: 0.58, yaw_inertia: 0.28, track_width: 0.34, body_length: 1.16, body_width: 0.48, body_height: 0.34, wheel_radius: 0.105, com_height: 0.20, front_cornering_stiffness: 24, rear_cornering_stiffness: 26, tire_longitudinal_stiffness: 33, suspension_stiffness: 250, suspension_damping: 7, roll_inertia: 0.34, pitch_inertia: 0.32, max_drive_torque: 2.1, v_max: 0.65 },
    "delivery-flat": { mass: 7.40, wheelbase: 0.56, yaw_inertia: 0.25, track_width: 0.34, body_length: 1.12, body_width: 0.48, body_height: 0.28, wheel_radius: 0.10, com_height: 0.16, front_cornering_stiffness: 25, rear_cornering_stiffness: 27, tire_longitudinal_stiffness: 34, suspension_stiffness: 240, suspension_damping: 7, roll_inertia: 0.30, pitch_inertia: 0.29, max_drive_torque: 2.0, v_max: 0.7 },
    truck: { mass: 10.00, wheelbase: 0.68, yaw_inertia: 0.48, track_width: 0.40, body_length: 1.35, body_width: 0.56, body_height: 0.40, wheel_radius: 0.12, com_height: 0.23, front_cornering_stiffness: 22, rear_cornering_stiffness: 24, tire_longitudinal_stiffness: 31, suspension_stiffness: 300, suspension_damping: 9, roll_inertia: 0.55, pitch_inertia: 0.50, max_drive_torque: 2.6, v_max: 0.55 },
    "truck-flat": { mass: 9.50, wheelbase: 0.66, yaw_inertia: 0.43, track_width: 0.39, body_length: 1.30, body_width: 0.55, body_height: 0.34, wheel_radius: 0.115, com_height: 0.20, front_cornering_stiffness: 23, rear_cornering_stiffness: 25, tire_longitudinal_stiffness: 32, suspension_stiffness: 290, suspension_damping: 9, roll_inertia: 0.50, pitch_inertia: 0.46, max_drive_torque: 2.5, v_max: 0.58 },
    ambulance: { mass: 9.00, wheelbase: 0.64, yaw_inertia: 0.40, track_width: 0.38, body_length: 1.28, body_width: 0.54, body_height: 0.42, wheel_radius: 0.115, com_height: 0.24, front_cornering_stiffness: 23, rear_cornering_stiffness: 25, tire_longitudinal_stiffness: 32, suspension_stiffness: 280, suspension_damping: 9, roll_inertia: 0.48, pitch_inertia: 0.45, max_drive_torque: 2.4, v_max: 0.6 },
    firetruck: { mass: 12.00, wheelbase: 0.72, yaw_inertia: 0.65, track_width: 0.43, body_length: 1.45, body_width: 0.60, body_height: 0.44, wheel_radius: 0.13, com_height: 0.25, front_cornering_stiffness: 20, rear_cornering_stiffness: 22, tire_longitudinal_stiffness: 29, suspension_stiffness: 340, suspension_damping: 11, roll_inertia: 0.75, pitch_inertia: 0.68, max_drive_torque: 3.0, v_max: 0.45 },
    "garbage-truck": { mass: 13.00, wheelbase: 0.74, yaw_inertia: 0.72, track_width: 0.45, body_length: 1.50, body_width: 0.62, body_height: 0.46, wheel_radius: 0.135, com_height: 0.27, front_cornering_stiffness: 19, rear_cornering_stiffness: 21, tire_longitudinal_stiffness: 28, suspension_stiffness: 360, suspension_damping: 12, roll_inertia: 0.85, pitch_inertia: 0.75, max_drive_torque: 3.2, v_max: 0.4 },
    police: { mass: 6.00, wheelbase: 0.50, yaw_inertia: 0.16, track_width: 0.30, body_length: 1.00, body_width: 0.41, body_height: 0.26, wheel_radius: 0.08, com_height: 0.15, front_cornering_stiffness: 36, rear_cornering_stiffness: 38, tire_longitudinal_stiffness: 42, suspension_stiffness: 205, suspension_damping: 5, roll_inertia: 0.19, pitch_inertia: 0.20, max_drive_torque: 2.0, v_max: 0.95 },
    tractor: { mass: 11.00, wheelbase: 0.70, yaw_inertia: 0.62, track_width: 0.42, body_length: 1.40, body_width: 0.58, body_height: 0.40, wheel_radius: 0.14, com_height: 0.23, front_cornering_stiffness: 18, rear_cornering_stiffness: 20, tire_longitudinal_stiffness: 27, suspension_stiffness: 320, suspension_damping: 10, roll_inertia: 0.72, pitch_inertia: 0.65, max_drive_torque: 3.0, v_max: 0.35 },
    "tractor-police": { mass: 10.50, wheelbase: 0.68, yaw_inertia: 0.58, track_width: 0.41, body_length: 1.36, body_width: 0.57, body_height: 0.40, wheel_radius: 0.135, com_height: 0.23, front_cornering_stiffness: 20, rear_cornering_stiffness: 22, tire_longitudinal_stiffness: 29, suspension_stiffness: 310, suspension_damping: 10, roll_inertia: 0.68, pitch_inertia: 0.62, max_drive_torque: 2.8, v_max: 0.4 },
    "tractor-shovel": { mass: 12.50, wheelbase: 0.74, yaw_inertia: 0.70, track_width: 0.46, body_length: 1.52, body_width: 0.64, body_height: 0.45, wheel_radius: 0.15, com_height: 0.27, front_cornering_stiffness: 17, rear_cornering_stiffness: 19, tire_longitudinal_stiffness: 26, suspension_stiffness: 350, suspension_damping: 12, roll_inertia: 0.82, pitch_inertia: 0.76, max_drive_torque: 3.4, v_max: 0.3 },
  };
  MODEL_PROFILES["cc0-sports-a"] = {
    ...MODEL_PROFILES.race,
    mass: 4.20,
    wheelbase: 0.45,
    yaw_inertia: 0.10,
    track_width: 0.28,
    body_length: 0.88,
    body_width: 0.37,
    body_height: 0.19,
    wheel_radius: 0.072,
    com_height: 0.105,
    front_cornering_stiffness: 34,
    rear_cornering_stiffness: 36,
    tire_longitudinal_stiffness: 38,
    suspension_stiffness: 165,
    suspension_damping: 4,
    roll_inertia: 0.10,
    pitch_inertia: 0.11,
    max_drive_torque: 1.55,
    v_max: 7.5,
  };
  MODEL_PROFILES["cc0-sports-b"] = {
    ...MODEL_PROFILES["cc0-sports-a"],
    mass: 4.55,
    wheelbase: 0.47,
    track_width: 0.29,
    body_length: 0.92,
    body_width: 0.38,
    body_height: 0.20,
    com_height: 0.11,
    front_cornering_stiffness: 36,
    rear_cornering_stiffness: 38,
    max_drive_torque: 1.7,
    v_max: 7.2,
  };
  MODEL_PROFILES["cc0-suv"] = {
    ...MODEL_PROFILES.suv,
    mass: 7.4,
    wheelbase: 0.53,
    track_width: 0.33,
    body_length: 1.04,
    body_width: 0.45,
    body_height: 0.30,
    wheel_radius: 0.095,
    com_height: 0.18,
    max_drive_torque: 2.15,
    v_max: 5.4,
  };
  for (const kart of ["kart-oobi", "kart-oodi", "kart-ooli", "kart-oopi", "kart-oozi"]) {
    MODEL_PROFILES[kart] = { ...MODEL_PROFILES.race, mass: 2.6, wheelbase: 0.29, yaw_inertia: 0.035, track_width: 0.19, body_length: 0.55, body_width: 0.25, body_height: 0.12, wheel_radius: 0.05, com_height: 0.075, front_cornering_stiffness: 22, rear_cornering_stiffness: 23, tire_longitudinal_stiffness: 28, suspension_stiffness: 80, suspension_damping: 2, roll_inertia: 0.04, pitch_inertia: 0.045, max_drive_torque: 0.9, v_max: 0.85 };
  }
  const MODEL_SPEED_LIMITS = {
    "cc0-sports-a": 7.5, "cc0-sports-b": 7.2, "cc0-suv": 5.4,
    race: 8.0, "race-future": 9.0, "hatchback-sports": 8.0, "sedan-sports": 7.5,
    sedan: 6.5, suv: 5.5, "suv-luxury": 5.5, taxi: 6.0, van: 4.8,
    delivery: 4.5, "delivery-flat": 4.8, truck: 3.8, "truck-flat": 4.0,
    ambulance: 4.8, firetruck: 3.2, "garbage-truck": 2.8, police: 7.5,
    tractor: 2.6, "tractor-police": 3.0, "tractor-shovel": 2.2,
    "kart-oobi": 7.0, "kart-oodi": 7.0, "kart-ooli": 7.0, "kart-oopi": 7.0, "kart-oozi": 7.0,
  };
  for (const [modelId, speed] of Object.entries(MODEL_SPEED_LIMITS)) {
    if (MODEL_PROFILES[modelId]) MODEL_PROFILES[modelId].v_max = speed;
  }
  // The F1TENTH centerlines are metric.  Keep the default race car aligned
  // with the reference platform rather than the earlier toy dimensions.
  Object.assign(MODEL_PROFILES.race, {
    mass: 3.74,
    wheelbase: 0.33,
    track_width: 0.27,
    body_length: 0.58,
    body_width: 0.31,
    body_height: 0.16,
    front_length: 0.15875,
    rear_length: 0.17145,
    wheel_radius: 0.055,
    com_height: 0.074,
    max_steering_angle: 0.4,
    v_max: 8.0,
  });

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x101418);
  scene.fog = new THREE.Fog(0x101418, 18, 70);
  const worldGroup = new THREE.Group();
  worldGroup.rotation.z = Math.PI / 2;
  scene.add(worldGroup);
  const windGroup = new THREE.Group();
  const windStreaks = [];
  for (let index = 0; index < 36; index += 1) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(new Float32Array(18), 3));
    const material = new THREE.LineBasicMaterial({
      color: 0x7dd3fc,
      transparent: true,
      opacity: 0.0,
      toneMapped: false,
    });
    const line = new THREE.Line(geometry, material);
    windGroup.add(line);
    windStreaks.push({
      line,
      phase: (index * 0.61803398875) % 1,
      lateral: ((index * 37) % 100) / 100 - 0.5,
      vertical: 0.25 + ((index * 19) % 100) / 100 * 1.8,
    });
  }
  windGroup.visible = false;
  worldGroup.add(raceTrackGroup, comparisonGroup, windGroup);

  const camera = new THREE.PerspectiveCamera(55, 1, 0.05, 1000);
  // The simulation/world use Z as up.  Three.js defaults to Y-up, which can
  // make a chase camera roll when the vehicle changes heading.
  camera.up.set(0, 0, 1);
  camera.position.set(-1.8, -2.8, 1.8);
  camera.lookAt(0, 0, 0);

  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  scene.add(new THREE.HemisphereLight(0xeef7ff, 0x222928, 1.2));
  const sun = new THREE.DirectionalLight(0xffffff, 1.8);
  sun.position.set(-4, -5, 8);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  scene.add(sun);

  const grid = new THREE.GridHelper(80, 80, 0x3b474b, 0x20282b);
  grid.rotation.x = Math.PI / 2;
  worldGroup.add(grid);
  const ground = new THREE.Mesh(
    new THREE.PlaneGeometry(90, 90),
    new THREE.MeshStandardMaterial({ color: 0x151b1e, roughness: 0.95, metalness: 0.0 })
  );
  ground.receiveShadow = true;
  worldGroup.add(ground);

  const terrainTextureCache = new Map();

  function terrainTexture(id) {
    if (terrainTextureCache.has(id)) return terrainTextureCache.get(id);
    const visual = TERRAIN_VISUALS[id] || TERRAIN_VISUALS.asphalt;
    const canvasTexture = document.createElement("canvas");
    canvasTexture.width = 256;
    canvasTexture.height = 256;
    const context = canvasTexture.getContext("2d");
    context.fillStyle = `#${visual.ground.toString(16).padStart(6, "0")}`;
    context.fillRect(0, 0, 256, 256);
    const colors = {
      asphalt: ["#263238", "#4a565a"], wet: ["#152a36", "#527180"],
      gravel: ["#81796a", "#b1a58e"], grass: ["#3b5b38", "#71925c"],
      mud: ["#4a3529", "#806248"], ice: ["#718f9d", "#c4e0e6"],
    }[visual.pattern] || ["#263238", "#4a565a"];
    for (let index = 0; index < 900; index += 1) {
      const x = (index * 47) % 256;
      const y = (index * 83) % 256;
      const radius = 1 + ((index * 13) % 4);
      context.fillStyle = colors[index % colors.length];
      context.globalAlpha = visual.pattern === "ice" ? 0.22 : 0.32;
      context.beginPath();
      context.arc(x, y, radius, 0, Math.PI * 2);
      context.fill();
    }
    context.globalAlpha = 1;
    const texture = new THREE.CanvasTexture(canvasTexture);
    texture.wrapS = THREE.RepeatWrapping;
    texture.wrapT = THREE.RepeatWrapping;
    texture.repeat.set(12, 12);
    texture.colorSpace = THREE.SRGBColorSpace;
    terrainTextureCache.set(id, texture);
    return texture;
  }

  function applyTerrainVisual(id, rerenderTrack = true) {
    activeTerrainId = TERRAIN_VISUALS[id] ? id : "asphalt";
    const visual = TERRAIN_VISUALS[activeTerrainId];
    ground.material.color.set(visual.ground);
    ground.material.map = terrainTexture(activeTerrainId);
    ground.material.roughness = visual.roughness;
    ground.material.needsUpdate = true;
    terrainType.value = activeTerrainId;
    terrainReadout.textContent = `${visual.label} · visual surface loaded`;
    if (rerenderTrack && activeRaceTrack) renderRaceTrack(activeRaceTrack);
  }

  applyTerrainVisual(activeTerrainId, false);

  const axes = makeAxes();
  worldGroup.add(axes);
  axes.visible = false;
  car = makeCar({});
  worldGroup.add(car.root);

  function toViewingFrame(vector) {
    return vector.clone().applyAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 2);
  }

  function makeAxes() {
    const group = new THREE.Group();
    group.add(makeAxis(0xff5d52, new THREE.Vector3(1, 0, 0)));
    group.add(makeAxis(0x70d36b, new THREE.Vector3(0, 1, 0)));
    group.add(makeAxis(0x67a7ff, new THREE.Vector3(0, 0, 1)));
    return group;
  }

  function makeAxis(color, direction) {
    const group = new THREE.Group();
    const length = 1.25;
    const shaft = new THREE.Mesh(
      new THREE.CylinderGeometry(0.015, 0.015, length, 12),
      new THREE.MeshBasicMaterial({ color })
    );
    const cone = new THREE.Mesh(
      new THREE.ConeGeometry(0.055, 0.16, 16),
      new THREE.MeshBasicMaterial({ color })
    );
    shaft.position.y = length / 2;
    cone.position.y = length + 0.08;
    group.add(shaft, cone);
    group.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction.clone().normalize());
    return group;
  }

  function meshBox(w, h, d, color, position) {
    const mesh = new THREE.Mesh(
      new THREE.BoxGeometry(w, h, d),
      new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.08 })
    );
    mesh.position.copy(position);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    return mesh;
  }

  function makeWheel(radius, width) {
    const mesh = new THREE.Mesh(
      new THREE.CylinderGeometry(radius, radius, width, 28),
      new THREE.MeshStandardMaterial({ color: 0x111315, roughness: 0.72 })
    );
    mesh.castShadow = true;
    return mesh;
  }

  function makeCar(vehicle) {
    const cfg = {
      wheelbase: vehicle.wheelbase || 0.324,
      front_length: vehicle.front_length || 0.162,
      rear_length: vehicle.rear_length || 0.162,
      track_width: vehicle.track_width || 0.22,
      body_length: vehicle.body_length || 0.62,
      body_width: vehicle.body_width || 0.28,
      body_height: vehicle.body_height || 0.14,
      wheel_radius: vehicle.wheel_radius || 0.055,
      com_height: vehicle.com_height || 0.09,
    };
    const root = new THREE.Group();
    const wheelRadius = Math.max(0.005, cfg.wheel_radius);
    const body = meshBox(
      cfg.body_length,
      cfg.body_width,
      cfg.body_height,
      0xe44f3f,
      new THREE.Vector3(0, 0, 0.01)
    );
    const deck = meshBox(
      cfg.body_length * 0.62,
      cfg.body_width * 0.84,
      cfg.body_height * 0.75,
      0x222b30,
      new THREE.Vector3(-cfg.body_length * 0.05, 0, cfg.body_height * 0.72)
    );
    const nose = meshBox(
      cfg.body_length * 0.22,
      cfg.body_width * 0.84,
      cfg.body_height * 0.68,
      0xf0d85b,
      new THREE.Vector3(cfg.body_length * 0.36, 0, cfg.body_height * 0.28)
    );
    root.add(body, deck, nose);

    const wheels = [];
    for (const x of [cfg.front_length, -cfg.rear_length]) {
      for (const y of [cfg.track_width / 2, -cfg.track_width / 2]) {
        const steeringGroup = new THREE.Group();
        steeringGroup.position.set(x, y, -cfg.com_height);
        const wheel = makeWheel(wheelRadius, Math.max(0.025, cfg.body_width * 0.22));
        steeringGroup.add(wheel);
        root.add(steeringGroup);
        wheels.push({ steeringGroup, wheel, front: x > 0 });
      }
    }
    const forceGroup = new THREE.Group();
    root.add(forceGroup);
    const primitiveParts = [body, deck, nose, ...wheels.map((wheel) => wheel.steeringGroup)];
    root.userData.geometryKey = [
      cfg.wheelbase, cfg.front_length, cfg.rear_length, cfg.track_width,
      cfg.body_length, cfg.body_width, cfg.body_height, cfg.wheel_radius, cfg.com_height,
    ].join(":");
    return {
      root, wheels, forceGroup, wheelRadius, primitiveParts,
      modelRoot: null, modelWheels: [], loadingModelId: null,
    };
  }

  function rebuildCar(vehicle) {
    const oldRoot = car.root;
    car = makeCar(vehicle || {});
    loadedModelId = null;
    worldGroup.remove(oldRoot);
    worldGroup.add(car.root);
    if (latestSnapshot) {
      updateSimulation(latestSnapshot);
      loadVehicleModel(selectedModel, vehicle || latestSnapshot.vehicle || {});
    }
  }

  function findModelNode(root, names) {
    let found = null;
    root.traverse((node) => {
      if (!found && names.includes(String(node.name).toLowerCase())) found = node;
    });
    return found;
  }

  function findModelNodeMatching(root, patterns, excluded = []) {
    let found = null;
    root.traverse((node) => {
      if (found || !node.name) return;
      const name = String(node.name).toLowerCase();
      if (patterns.some((pattern) => name.includes(pattern)) && !excluded.some((pattern) => name.includes(pattern))) {
        found = node;
      }
    });
    return found;
  }

  function modelWheelNodes(root) {
    const frontLeft = findModelNode(root, ["wheel-front-left"])
      || findModelNodeMatching(root, ["frontleftwheel", "front-left", "front_left"]);
    const frontRight = findModelNode(root, ["wheel-front-right"])
      || findModelNodeMatching(root, ["frontrightwheel", "front-right", "front_right"]);
    const rearLeft = findModelNode(root, ["wheel-back-left"])
      || findModelNodeMatching(root, ["backleftwheel", "rearleftwheel", "rear-left", "rear_left", "back-left", "back_left"])
      || findModelNodeMatching(root, ["backwheels", "rearwheels"], ["front"]);
    const rearRight = findModelNode(root, ["wheel-back-right"])
      || findModelNodeMatching(root, ["backrightwheel", "rearrightwheel", "rear-right", "rear_right", "back-right", "back_right"])
      || findModelNodeMatching(root, ["backwheels", "rearwheels"], ["front"]);
    return [frontLeft, frontRight, rearLeft, rearRight];
  }

  function prepareLoadedModel(model, modelId) {
    const asset = MODEL_ASSETS[modelId];
    const sourceAxis = asset?.axis || "x-width-z-forward";
    if (sourceAxis === "x-width-z-forward") {
      // Both the Kenney fallback GLBs and the imported car GLBs use
      // X=width, Y=up, Z=length. Convert to the viewer's X=forward,
      // Y=left, Z=up convention used by the physics state. Without this
      // conversion the front/rear wheelbase is read from the source width
      // axis, producing a sideways vehicle and an incorrect scale.
      model.applyMatrix4(new THREE.Matrix4().set(
        0, 0, 1, 0,
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 0, 1
      ));
    }
    model.updateMatrixWorld(true);
    model.traverse((node) => {
      if (!node.isMesh) return;
      node.castShadow = false;
      node.receiveShadow = true;
      if (node.material) {
        const materials = Array.isArray(node.material) ? node.material : [node.material];
        materials.forEach((material) => {
          material.envMapIntensity = 0.8;
          material.needsUpdate = true;
        });
      }
    });
    return model;
  }

  function createModelWheelRigs(model, wheelNodes) {
    const seen = new Set();
    return wheelNodes.map((node, index) => {
      if (!node || seen.has(node)) return null;
      seen.add(node);

      // Some imported GLBs bake wheel vertices in vehicle coordinates while
      // leaving the wheel node at the model origin. Put two pivots at the
      // measured wheel center: the outer pivot steers around source-Y (up),
      // and the inner pivot spins around source-X (the wheel axle). This keeps
      // the mesh stationary in world space when the rig is created.
      const worldCenter = new THREE.Box3().setFromObject(node).getCenter(new THREE.Vector3());
      const nodeWorld = node.matrixWorld.clone();
      const steeringPivot = new THREE.Group();
      steeringPivot.name = `${node.name || "wheel"}-steering-pivot`;
      steeringPivot.position.copy(model.worldToLocal(worldCenter.clone()));
      model.add(steeringPivot);
      const spinPivot = new THREE.Group();
      spinPivot.name = `${node.name || "wheel"}-spin-pivot`;
      steeringPivot.add(spinPivot);
      model.updateMatrixWorld(true);

      node.parent?.remove(node);
      spinPivot.add(node);
      spinPivot.updateMatrixWorld(true);
      const localNodeMatrix = spinPivot.matrixWorld.clone().invert().multiply(nodeWorld);
      localNodeMatrix.decompose(node.position, node.quaternion, node.scale);
      node.updateMatrix();
      node.updateMatrixWorld(true);

      return {
        front: index < 2,
        steeringPivot,
        spinPivot,
        node,
      };
    });
  }

  function loadVehicleModel(modelId, vehicle) {
    if (!THREE.GLTFLoader || !modelId || car.loadingModelId === modelId || loadedModelId === modelId) return;
    car.loadingModelId = modelId;
    const loader = new THREE.GLTFLoader();
    const asset = MODEL_ASSETS[modelId];
    loader.load(
      asset?.url || `./assets/kenney_car/models/${encodeURIComponent(modelId)}.glb`,
      (gltf) => {
        if (selectedModel !== modelId) {
          car.loadingModelId = null;
          return;
        }
        const model = prepareLoadedModel(gltf.scene, modelId);
        const wheelNodes = modelWheelNodes(model);
        const frontNode = wheelNodes[0] || wheelNodes[1];
        const rearNode = wheelNodes[2] || wheelNodes[3];
        const frontPosition = new THREE.Vector3();
        const rearPosition = new THREE.Vector3();
        frontNode?.getWorldPosition(frontPosition);
        rearNode?.getWorldPosition(rearPosition);
        const rawWheelbase = Math.abs(frontPosition.x - rearPosition.x);
        const desiredWheelbase = Number(vehicle.wheelbase || 0.324);
        const boundsBeforeScale = new THREE.Box3().setFromObject(model);
        const sourceLength = Math.max(boundsBeforeScale.max.x - boundsBeforeScale.min.x, 1e-5);
        const desiredLength = Number(vehicle.body_length || desiredWheelbase * 1.9);
        const scale = rawWheelbase > 1e-5
          ? desiredWheelbase / rawWheelbase
          : desiredLength / sourceLength;
        model.scale.setScalar(scale);
        model.updateMatrixWorld(true);
        let bounds = new THREE.Box3().setFromObject(model);
        const center = new THREE.Vector3();
        bounds.getCenter(center);
        model.position.x -= center.x;
        model.position.y -= center.y;
        bounds = new THREE.Box3().setFromObject(model);
        const desiredBottom = -Number(vehicle.com_height || 0.09) - Number(vehicle.wheel_radius || 0.055);
        model.position.z += desiredBottom - bounds.min.z;
        model.updateMatrixWorld(true);

        if (car.modelRoot) car.root.remove(car.modelRoot);
        car.modelRoot = model;
        car.modelWheels = createModelWheelRigs(model, modelWheelNodes(model));
        car.root.add(model);
        car.primitiveParts.forEach((part) => { part.visible = false; });
        car.loadingModelId = null;
        loadedModelId = modelId;
        sourceLabel.textContent = asset?.label || `Kenney Car Kit · ${modelId}`;
      },
      undefined,
      () => {
        car.loadingModelId = null;
        car.primitiveParts.forEach((part) => { part.visible = true; });
        sourceLabel.textContent = "Procedural fallback model";
      }
    );
  }

  function resize() {
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    renderer.setSize(width, height, false);
    camera.aspect = width / Math.max(height, 1);
    camera.updateProjectionMatrix();
  }

  window.addEventListener("resize", resize);
  resize();

  function connectSimulation() {
    if (location.protocol === "file:") {
      statusReadout.textContent = "run sim_server.py";
      sourceLabel.textContent = "WebSocket unavailable from file://";
      return;
    }
    const protocol = location.protocol === "https:" ? "wss" : "ws";
    socket = new WebSocket(`${protocol}://${location.host}/ws`);
    socket.addEventListener("open", () => {
      statusReadout.textContent = "simulation connected";
      sourceLabel.textContent = "Python 6-DOF vehicle simulator";
      send({ type: "hello" });
      applyModel(selectedModel, { pause: false });
      resetToRaceTrackStart();
    });
    socket.addEventListener("close", () => {
      statusReadout.textContent = "simulation disconnected";
    });
    socket.addEventListener("error", () => {
      statusReadout.textContent = "server unavailable";
    });
    socket.addEventListener("message", (event) => {
      try {
        const message = JSON.parse(event.data);
        if (message.type === "snapshot") updateSimulation(message);
        if (message.type === "comparison") updateComparison(message);
      } catch (error) {
        console.warn("Invalid simulation message", error);
      }
    });
  }

  function send(message) {
    if (socket && socket.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message));
  }

  function clearGroup(group) {
    while (group.children.length) {
      const child = group.children[0];
      group.remove(child);
      if (child.geometry) child.geometry.dispose();
      if (child.material) {
        if (Array.isArray(child.material)) child.material.forEach((material) => material.dispose());
        else child.material.dispose();
      }
    }
  }

  function prepareRaceTrack(rawTrack) {
    const points = rawTrack.points.map((point) => {
      const heading = Number(point.heading) || 0;
      const left = new THREE.Vector3(-Math.sin(heading), Math.cos(heading), 0);
      return {
        ...point,
        heading,
        curvature: Number(point.curvature) || 0,
        benchmarkSpeed: Number(point.benchmark_speed) || 8.0,
        position: new THREE.Vector3(Number(point.x), Number(point.y), 0),
        left,
      };
    });
    return { ...rawTrack, points };
  }

  function renderRaceTrack(track) {
    clearGroup(raceTrackGroup);
    const points = track.points;
    const positions = [];
    const indices = [];
    for (let index = 0; index < points.length; index += 1) {
      const point = points[index];
      const next = points[(index + 1) % points.length];
      const left = point.position.clone().addScaledVector(point.left, Number(point.width_left) || 0.9);
      const right = point.position.clone().addScaledVector(point.left, -(Number(point.width_right) || 0.9));
      const nextLeft = next.position.clone().addScaledVector(next.left, Number(next.width_left) || 0.9);
      const nextRight = next.position.clone().addScaledVector(next.left, -(Number(next.width_right) || 0.9));
      const base = positions.length / 3;
      positions.push(left.x, left.y, 0.025, right.x, right.y, 0.025, nextLeft.x, nextLeft.y, 0.025, nextRight.x, nextRight.y, 0.025);
      indices.push(base, base + 1, base + 2, base + 1, base + 3, base + 2);
    }
    const roadGeometry = new THREE.BufferGeometry();
    roadGeometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    roadGeometry.setIndex(indices);
    roadGeometry.computeVertexNormals();
    const road = new THREE.Mesh(
      roadGeometry,
      new THREE.MeshStandardMaterial({
        color: TERRAIN_VISUALS[activeTerrainId].road,
        roughness: TERRAIN_VISUALS[activeTerrainId].roughness,
        metalness: activeTerrainId === "wet-asphalt" ? 0.18 : 0.02,
        side: THREE.DoubleSide,
      })
    );
    road.receiveShadow = true;
    raceTrackGroup.add(road);

    const centerPoints = points.map((point) => point.position.clone().setZ(0.045));
    centerPoints.push(centerPoints[0].clone());
    raceTrackGroup.add(new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(centerPoints),
      new THREE.LineBasicMaterial({ color: 0x77848a, transparent: true, opacity: 0.65 })
    ));
    for (const side of ["left", "right"]) {
      const edgePoints = points.map((point) => point.position.clone().addScaledVector(
        point.left,
        side === "left" ? Number(point.width_left) || 0.9 : -(Number(point.width_right) || 0.9)
      ).setZ(0.055));
      edgePoints.push(edgePoints[0].clone());
      raceTrackGroup.add(new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(edgePoints),
        new THREE.LineBasicMaterial({ color: 0xdbe85f, transparent: true, opacity: 0.8 })
      ));
    }
    const start = points[0].position.clone().setZ(0.065);
    const marker = new THREE.Mesh(
      new THREE.TorusGeometry(0.28, 0.025, 8, 32),
      new THREE.MeshBasicMaterial({ color: 0xff8c69 })
    );
    marker.rotation.x = 0;
    marker.position.copy(start);
    raceTrackGroup.add(marker);
  }

  function loadRaceTracks() {
    fetch("./api/f1tenth/paths")
      .then((response) => {
        if (!response.ok) throw new Error(`track catalog request failed: ${response.status}`);
        return response.json();
      })
      .then((catalog) => {
        trackCatalog = catalog.tracks || {};
        if (!trackCatalog[raceTrack.value]) raceTrack.value = Object.keys(trackCatalog)[0] || "aut";
        loadRaceTrack(raceTrack.value);
      })
      .catch((error) => {
        trackReadout.textContent = "F1TENTH paths unavailable";
        console.warn("Unable to load F1TENTH tracks", error);
      });
  }

  function loadRaceTrack(trackId) {
    if (!trackCatalog[trackId]) return;
    setComparisonView(false);
    clearGroup(comparisonGroup);
    activeRaceTrack = prepareRaceTrack(trackCatalog[trackId]);
    updateTrackViewBounds(activeRaceTrack);
    pathFollowerIndex = null;
    lastPathControllerTime = -Infinity;
    raceTrack.value = trackId;
    tracksideAnchor = makeTracksideAnchor(activeRaceTrack);
    renderRaceTrack(activeRaceTrack);
    trackReadout.textContent = `${activeRaceTrack.name} · automatic race-path following · ${Number(activeRaceTrack.length).toFixed(1)} m`;
    resetToRaceTrackStart();
  }

  function loadTerrain(id) {
    if (!TERRAIN_VISUALS[id]) return;
    applyTerrainVisual(id);
    send({ type: "play", value: false });
    send({ type: "terrain", terrain_id: id });
    resetToRaceTrackStart();
    trackReadout.textContent = activeRaceTrack
      ? `${activeRaceTrack.name} · ${TERRAIN_VISUALS[id].label} · automatic race-path following`
      : `${TERRAIN_VISUALS[id].label} selected`;
  }

  function updateTrackViewBounds(track) {
    const bounds = new THREE.Box3();
    for (const point of track.points) {
      bounds.expandByPoint(point.position);
      bounds.expandByPoint(point.position.clone().addScaledVector(point.left, Number(point.width_left) || 0.9));
      bounds.expandByPoint(point.position.clone().addScaledVector(point.left, -(Number(point.width_right) || 0.9)));
    }
    bounds.getCenter(trackViewCenter).setZ(0);
    trackViewSize.set(
      Math.max(bounds.max.x - bounds.min.x, 1),
      Math.max(bounds.max.y - bounds.min.y, 1),
    );
  }

  function makeTracksideAnchor(track) {
    const point = track.points[Math.floor(track.points.length * 0.25) % track.points.length];
    const offset = Math.max(Number(point.width_left) || 0.9, 0.9) + 3.2;
    return {
      position: point.position.clone().addScaledVector(point.left, offset).setZ(2.0),
      target: point.position.clone().setZ(0.35),
    };
  }

  function resetToRaceTrackStart() {
    pathFollowerIndex = null;
    lastPathControllerTime = -Infinity;
    send({ type: "reset" });
    if (!activeRaceTrack || !activeRaceTrack.points.length) return;
    const start = activeRaceTrack.points[0];
    send({ type: "pose", x: start.x, y: start.y, heading: start.heading });
  }

  function wrapAngle(angle) {
    return Math.atan2(Math.sin(angle), Math.cos(angle));
  }

  function nearestRacePoint(x, y) {
    if (!activeRaceTrack) return null;
    const points = activeRaceTrack.points;
    let nearest = null;
    let bestDistance = Infinity;
    const candidates = [];
    if (pathFollowerIndex === null) {
      for (let index = 0; index < points.length; index += 1) candidates.push(index);
    } else {
      // Stay near the current lap position.  A global nearest-point lookup
      // can jump to a different part of a tight/overlapping circuit and make
      // a controller appear to drive straight through the boundary.
      for (let offset = -30; offset <= 180; offset += 1) {
        candidates.push((pathFollowerIndex + offset + points.length) % points.length);
      }
    }
    for (const index of candidates) {
      const point = points[index];
      const distance = Math.hypot(point.x - x, point.y - y);
      if (distance < bestDistance) {
        bestDistance = distance;
        nearest = { point, index, distance };
      }
    }
    if (nearest && nearest.distance > 2.0) {
      // Recover if the vehicle has already left the local search window.
      for (let index = 0; index < points.length; index += 1) {
        const point = points[index];
        const distance = Math.hypot(point.x - x, point.y - y);
        if (distance < bestDistance) {
          bestDistance = distance;
          nearest = { point, index, distance };
        }
      }
    }
    if (nearest) pathFollowerIndex = nearest.index;
    return nearest;
  }

  function lookaheadRacePoint(nearest, lookahead) {
    const points = activeRaceTrack.points;
    const spacing = Math.max(Number(activeRaceTrack.point_spacing) || 0.2, 0.05);
    const offset = Math.max(2, Math.round(lookahead / spacing));
    return points[(nearest.index + offset) % points.length];
  }

  function updateRacePathFollower(snapshot) {
    // A reset arrives before the browser's start-pose message. Do not issue a
    // steering command from the server's temporary origin state; doing so can
    // leave a stale cornering command active when playback is resumed.
    if (!activeRaceTrack || !snapshot.state || !snapshot.playing || snapshot.pose_ready === false) return;
    if (Number(snapshot.time) - lastPathControllerTime < 0.05) return;
    lastPathControllerTime = Number(snapshot.time);
    const state = snapshot.state;
    const vehicle = snapshot.vehicle || {};
    const nearest = nearestRacePoint(Number(state.x), Number(state.y));
    if (!nearest) return;
    const speed = Math.max(0, Math.abs(Number(state.vx) || 0));
    const lookahead = Math.max(0.65, Math.min(2.8, 0.65 + speed * 0.22));
    const target = lookaheadRacePoint(nearest, lookahead);
    const targetAngle = Math.atan2(target.y - state.y, target.x - state.x);
    const heading = Number(state.heading || 0);
    const alpha = wrapAngle(targetAngle - heading);
    const wheelbase = Number(vehicle.wheelbase) || 0.324;
    const purePursuitSteering = Math.atan2(2.0 * wheelbase * Math.sin(alpha), lookahead);
    const pathHeading = Number(nearest.point.heading) || 0;
    const headingError = wrapAngle(pathHeading - heading);
    // Signed error is positive when the vehicle is left of the path.  The
    // negative Stanley term therefore steers it back toward the centerline,
    // including when it has already drifted outside the road ribbon.
    const lateralError =
      -Math.sin(pathHeading) * (Number(state.x) - nearest.point.x) +
      Math.cos(pathHeading) * (Number(state.y) - nearest.point.y);
    const correction = Math.atan2(2.0 * lateralError, speed + 0.5);
    const steering = purePursuitSteering + 0.25 * headingError - correction;
    const maxSteering = Number(vehicle.max_steering_angle) || 0.396;
    const steeringCommand = Math.max(-maxSteering, Math.min(maxSteering, steering));
    const benchmarkMu = Number(activeRaceTrack.speed_profile?.mu) || 0.7;
    const gripScale = Math.sqrt(Math.max(0.1, Number(snapshot.mu) || benchmarkMu) / benchmarkMu);
    // F1TENTH GlobalPurePursuit uses a friction-limited speed based on the
    // commanded steering angle.  Preview the next 1.5 m as well, so braking
    // begins before a corner rather than after the vehicle reaches it.
    const spacing = Math.max(Number(activeRaceTrack.point_spacing) || 0.2, 0.05);
    const previewPoints = Math.max(2, Math.ceil(1.5 / spacing));
    let previewSpeed = Number.POSITIVE_INFINITY;
    for (let offset = 0; offset <= previewPoints; offset += 1) {
      const point = activeRaceTrack.points[(nearest.index + offset) % activeRaceTrack.points.length];
      previewSpeed = Math.min(previewSpeed, Number(point.benchmarkSpeed) || 8.0);
    }
    const frictionLimit = 1.5;
    const steeringSpeedLimit = Math.abs(steeringCommand) < 0.03
      ? 8.0
      : Math.sqrt(frictionLimit * 9.81 * wheelbase / Math.max(Math.tan(Math.abs(steeringCommand)), 1e-3));
    // The six-DOF viewer plant needs a modest calibration margin relative to
    // the ideal benchmark planner, while retaining its changing speed shape.
    const profileSpeed = Math.min(previewSpeed, steeringSpeedLimit, 8.0) * gripScale * 0.70;
    const requestedSpeed = Math.max(0.1, Number(vehicle.v_max) || 0.1);
    const targetSpeedForPath = Math.min(
      requestedSpeed,
      profileSpeed,
      Number(vehicle.v_max) || requestedSpeed,
    );
    send({
      type: "control",
      target_speed: targetSpeedForPath,
      steering: steeringCommand,
    });
  }

  function setCameraMode(nextMode) {
    if (comparisonView && nextMode !== "top") setComparisonView(false);
    cameraMode = ["chase", "top", "trackside"].includes(nextMode) ? nextMode : "chase";
    cameraModeInput.value = cameraMode;
  }

  function comparisonCameraHeight() {
    const horizontalSpan = trackViewSize.x / Math.max(camera.aspect, 0.1);
    const span = Math.max(horizontalSpan, trackViewSize.y);
    return Math.max(8, span / (2 * Math.tan(THREE.MathUtils.degToRad(camera.fov / 2))) * 1.25);
  }

  function focusComparisonTrack() {
    const center = trackViewCenter.clone().setZ(0);
    const desired = center.clone().setZ(comparisonCameraHeight());
    camera.position.copy(toViewingFrame(desired));
    cameraLookTarget.copy(toViewingFrame(center));
  }

  function setComparisonView(enabled) {
    comparisonView = Boolean(enabled && activeRaceTrack);
    if (!comparisonView) stopComparisonPlayback();
    if (car) car.root.visible = !comparisonView;
    axes.visible = false;
    if (trailLine) trailLine.visible = !comparisonView;
    comparisonGroup.visible = comparisonView;
    windGroup.visible = !comparisonView && activeWindSpeed >= 0.2;
    comparisonLegend.hidden = !comparisonView;
    if (comparisonView) {
      cameraMode = "top";
      cameraModeInput.value = "top";
      focusComparisonTrack();
    }
  }

  function updateAutoCamera() {
    if (!car || !latestSnapshot || !latestSnapshot.state) return;
    if (comparisonView && activeRaceTrack) {
      const center = trackViewCenter.clone().setZ(0);
      const desired = center.clone().setZ(comparisonCameraHeight());
      camera.position.lerp(toViewingFrame(desired), 0.18);
      cameraLookTarget.lerp(toViewingFrame(center), 0.18);
      camera.lookAt(cameraLookTarget);
      return;
    }
    const state = latestSnapshot.state;
    const position = car.root.position.clone();
    const heading = Number(state.heading || 0);
    const forward = new THREE.Vector3(Math.cos(heading), Math.sin(heading), 0);
    let desired;
    let target;
    if (cameraMode === "chase") {
      desired = position.clone().addScaledVector(forward, -3.0).setZ(position.z + 1.55);
      target = position.clone().addScaledVector(forward, 1.25).setZ(position.z + 0.3);
    } else if (cameraMode === "top") {
      desired = position.clone().add(new THREE.Vector3(0, 0, 10.0));
      target = position.clone().setZ(0);
    } else {
      const anchor = tracksideAnchor || { position: position.clone().add(new THREE.Vector3(-4, 3, 2.2)), target: position.clone() };
      desired = anchor.position;
      target = anchor.target;
    }
    camera.position.lerp(toViewingFrame(desired), 0.12);
    cameraLookTarget.lerp(toViewingFrame(target), 0.12);
    camera.lookAt(cameraLookTarget);
  }

  simPlayPause.addEventListener("click", () => {
    const playing = !(latestSnapshot && latestSnapshot.playing);
    send({ type: "play", value: playing });
    simPlayPause.textContent = playing ? "Pause" : "Play";
  });
  simReset.addEventListener("click", () => {
    setComparisonView(false);
    clearGroup(comparisonGroup);
    send({ type: "play", value: false });
    resetToRaceTrackStart();
    simPlayPause.textContent = "Play";
  });
  muInput.addEventListener("change", sendParameters);
  parameterInputs.forEach((input) => input.addEventListener("change", () => {
    if (["wind_direction_deg", "aero_drag", "aero_side_drag", "air_density"].includes(input.dataset.parameter)) {
      activeWindProfileId = "custom";
      windProfile.value = "custom";
    }
    sendParameters();
  }));
  modelPreset.addEventListener("change", () => applyModel(modelPreset.value));
  windProfile.addEventListener("change", () => applyWindProfile(windProfile.value));
  raceTrack.addEventListener("change", () => loadRaceTrack(raceTrack.value));
  loadRaceTrackButton.addEventListener("click", () => loadRaceTrack(raceTrack.value));
  cameraModeInput.addEventListener("change", () => setCameraMode(cameraModeInput.value));
  terrainType.addEventListener("change", () => loadTerrain(terrainType.value));
  compareModels.addEventListener("click", requestModelComparison);
  compareTerrains.addEventListener("click", requestTerrainComparison);
  compareModel1.addEventListener("change", () => { if (compareModel1.value === compareModel2.value) compareModel2.value = selectedModel; });
  compareModel2.addEventListener("change", () => { if (compareModel2.value === compareModel1.value) compareModel1.value = selectedModel; });
  compareTerrain1.addEventListener("change", () => { if (compareTerrain1.value === compareTerrain2.value) compareTerrain2.value = activeTerrainId; });
  compareTerrain2.addEventListener("change", () => { if (compareTerrain2.value === compareTerrain1.value) compareTerrain1.value = activeTerrainId; });
  forceInputs.forEach((input) => input.addEventListener("change", () => {
    if (latestSnapshot) updateForceArrows(latestSnapshot);
  }));

  function applyWindProfile(name, options = {}) {
    const profile = WIND_PROFILES[name];
    if (!profile) {
      activeWindProfileId = "custom";
      windProfile.value = "custom";
      return;
    }
    activeWindProfileId = name;
    activeWindSpeed = profile.speed;
    windProfile.value = name;
    for (const parameter of ["wind_direction_deg", "aero_drag", "aero_side_drag", "air_density"]) {
      const input = document.querySelector(`[data-parameter="${parameter}"]`);
      if (input && Object.prototype.hasOwnProperty.call(profile, parameter)) input.value = profile[parameter];
    }
    if (options.pause !== false) {
      send({ type: "play", value: false });
      simPlayPause.textContent = "Play";
    }
    sendParameters();
    if (options.reset !== false) resetToRaceTrackStart();
  }

  function sendParameters() {
    const vehicle = {};
    parameterInputs.forEach((input) => {
      if (input.value !== "") {
        const value = Number(input.value);
        vehicle[input.dataset.parameter] = value;
        if (MODEL_PROFILES[selectedModel]) MODEL_PROFILES[selectedModel][input.dataset.parameter] = value;
      }
    });
    // Wind speed is selected as a named profile rather than as an ordinary
    // parameter field, so it is sent explicitly with the editable values.
    vehicle.wind_speed = activeWindSpeed;
    send({ type: "parameters", model_id: selectedModel, vehicle, mu: Number(muInput.value) });
  }

  function applyModel(name, options = {}) {
    selectedModel = MODEL_PROFILES[name] ? name : "race";
    const pause = options.pause !== false;
    setComparisonView(false);
    clearGroup(comparisonGroup);
    if (pause) {
      send({ type: "play", value: false });
      simPlayPause.textContent = "Play";
    }
    loadedModelId = null;
    const preset = MODEL_PROFILES[selectedModel];
    for (const input of parameterInputs) {
      if (Object.prototype.hasOwnProperty.call(preset, input.dataset.parameter)) {
        input.value = preset[input.dataset.parameter];
      }
    }
    sendParameters();
    resetToRaceTrackStart();
    loadVehicleModel(selectedModel, preset);
  }

  function populateComparisonModels() {
    const options = [...modelPreset.options].map((option) => ({ value: option.value, label: option.textContent }));
    for (const select of [compareModel1, compareModel2]) {
      select.replaceChildren(...options.map(({ value, label }) => {
        const option = document.createElement("option");
        option.value = value;
        option.textContent = label;
        return option;
      }));
    }
    compareModel1.value = "race";
    compareModel2.value = "sedan";
    const terrainOptions = [...terrainType.options].map((option) => ({ value: option.value, label: option.textContent }));
    for (const select of [compareTerrain1, compareTerrain2]) {
      select.replaceChildren(...terrainOptions.map(({ value, label }) => {
        const option = document.createElement("option");
        option.value = value;
        option.textContent = label;
        return option;
      }));
    }
    compareTerrain1.value = activeTerrainId;
    compareTerrain2.value = activeTerrainId === "gravel" ? "asphalt" : "gravel";
  }

  function beginComparison(variants, profiles, statusText) {
    if (!activeRaceTrack) {
      comparisonReadout.textContent = "Load an F1TENTH race track first";
      return;
    }
    clearGroup(comparisonGroup);
    setComparisonView(true);
    send({
      type: "compare",
      variants,
      vehicle_profiles: profiles,
      track_id: activeRaceTrack.id,
      follow_path: true,
      target_speed: 8.0,
      steps: 1200,
    });
    comparisonReadout.textContent = `${activeRaceTrack.name} · ${statusText}`;
  }

  function requestModelComparison() {
    const comparisonModels = [compareModel1.value, compareModel2.value]
      .filter((name, index, values) => name && values.indexOf(name) === index);
    if (comparisonModels.length !== 2) {
      comparisonReadout.textContent = "Choose two different models";
      return;
    }
    const profiles = Object.fromEntries(
      comparisonModels.map((name) => [name, { ...(MODEL_PROFILES[name] || {}) }])
    );
    beginComparison(
      comparisonModels.map((modelId) => ({
        model_id: modelId,
        terrain_id: activeTerrainId,
        label: modelPreset.querySelector(`option[value="${modelId}"]`)?.textContent || modelId,
      })),
      profiles,
      "comparing model trajectories...",
    );
  }

  function requestTerrainComparison() {
    if (compareTerrain1.value === compareTerrain2.value) {
      comparisonReadout.textContent = "Choose two different terrains";
      return;
    }
    const modelLabel = modelPreset.options[modelPreset.selectedIndex]?.textContent || selectedModel;
    const profiles = { [selectedModel]: { ...(MODEL_PROFILES[selectedModel] || {}) } };
    beginComparison(
      [compareTerrain1.value, compareTerrain2.value].map((terrainId) => ({
        model_id: selectedModel,
        terrain_id: terrainId,
        label: `${modelLabel} · ${terrainType.querySelector(`option[value="${terrainId}"]`)?.textContent || terrainId}`,
      })),
      profiles,
      "comparing terrain trajectories...",
    );
  }

  function updateSimulation(snapshot) {
    latestSnapshot = snapshot;
    const state = snapshot.state;
    const vehicle = snapshot.vehicle || {};
    const modelId = snapshot.vehicle_model || selectedModel;
    if (modelId !== selectedModel && MODEL_PROFILES[modelId]) {
      selectedModel = modelId;
      modelPreset.value = modelId;
      loadedModelId = null;
    }
    const geometryKey = [
      vehicle.wheelbase, vehicle.front_length, vehicle.rear_length, vehicle.track_width,
      vehicle.body_length, vehicle.body_width, vehicle.body_height, vehicle.wheel_radius, vehicle.com_height,
    ].join(":");
    if (car.root.userData.geometryKey !== geometryKey) {
      rebuildCar(vehicle);
      return;
    }
    const visualZ = Number.isFinite(Number(state.z))
      ? Number(state.z) : Number(vehicle.com_height || 0.09);
    car.root.position.set(state.x, state.y, visualZ);
    car.root.rotation.set(state.roll || 0, state.pitch || 0, state.heading || 0, "XYZ");
    axes.position.copy(car.root.position);
    axes.quaternion.copy(car.root.quaternion);
    for (const [index, wheel] of car.wheels.entries()) {
      wheel.steeringGroup.rotation.z = wheel.front ? state.steering : 0;
      const wheelName = ["fl", "fr", "rl", "rr"][index];
      const angularSpeed = Number.isFinite(Number(state[`omega_${wheelName}`]))
        ? Number(state[`omega_${wheelName}`]) : state.vx / Math.max(car.wheelRadius, 1e-4);
      wheel.wheel.rotation.y += angularSpeed * (1 / 30);
    }
    for (const [index, wheel] of car.modelWheels.entries()) {
      if (!wheel) continue;
      const wheelName = ["fl", "fr", "rl", "rr"][index];
      const angularSpeed = Number.isFinite(Number(state[`omega_${wheelName}`]))
        ? Number(state[`omega_${wheelName}`]) : state.vx / Math.max(car.wheelRadius, 1e-4);
      // The loaded model is stored in source coordinates beneath the axis
      // conversion: source-Y is vehicle-up and source-X is the wheel axle.
      if (wheel.front) wheel.steeringPivot.rotation.y = state.steering || 0;
      wheel.spinPivot.rotation.x += angularSpeed * (1 / 30);
    }
    if (loadedModelId !== modelId) loadVehicleModel(modelId, vehicle);
    updateForceArrows(snapshot);
    updateReadouts(snapshot);
    for (const input of parameterInputs) {
      const value = vehicle[input.dataset.parameter];
      if (Number.isFinite(Number(value)) && document.activeElement !== input) input.value = value;
    }
    if (document.activeElement !== muInput) {
      muInput.value = Number(snapshot.surface_mu ?? snapshot.mu).toFixed(2);
    }
    if (snapshot.terrain && TERRAIN_VISUALS[snapshot.terrain.id]) {
      const terrainChanged = activeTerrainId !== snapshot.terrain.id;
      if (terrainChanged) applyTerrainVisual(snapshot.terrain.id);
      terrainReadout.textContent = `${snapshot.terrain.label} · effective μ ${Number(snapshot.mu).toFixed(2)}`;
    }
    simPlayPause.textContent = snapshot.playing ? "Pause" : "Play";
    updateRacePathFollower(snapshot);
  }

  function stopComparisonPlayback() {
    comparisonPlayback = null;
  }

  function comparisonMetricsText(message, suffix = "") {
    const metrics = message.trajectories.map((trajectory) => {
      const maxSpeed = Number(trajectory.max_speed);
      const margin = Number(trajectory.min_track_margin);
      const speedLabel = Number.isFinite(maxSpeed) ? `max ${maxSpeed.toFixed(1)} m/s` : "trajectory";
      const marginLabel = Number.isFinite(margin) ? `margin ${margin.toFixed(2)} m` : "";
      const statusLabel = trajectory.completed === false ? ", incomplete" : "";
      const label = trajectory.label || trajectory.model_id;
      return `${label}: ${speedLabel}${marginLabel ? `, ${marginLabel}` : ""}${statusLabel}`;
    });
    return `${message.track_id || "track"}${suffix} · ${metrics.join(" · ")}`;
  }

  function updateComparison(message) {
    stopComparisonPlayback();
    clearGroup(comparisonGroup);
    const colors = [0xdbe85f, 0x67a7ff, 0xff8c69, 0x70d36b, 0xc084fc, 0x8be9fd];
    const trajectories = [];
    comparisonLegendItems.replaceChildren();
    for (const [index, trajectory] of message.trajectories.entries()) {
      const samples = Array.isArray(trajectory.points) ? trajectory.points : [];
      if (!samples.length) continue;
      const color = colors[index % colors.length];
      const legendItem = document.createElement("div");
      legendItem.className = "comparison-legend-item";
      const swatch = document.createElement("span");
      swatch.className = "comparison-legend-swatch";
      swatch.style.backgroundColor = `#${color.toString(16).padStart(6, "0")}`;
      const label = document.createElement("span");
      label.textContent = trajectory.label || trajectory.model_id || `Trajectory ${index + 1}`;
      legendItem.append(swatch, label);
      comparisonLegendItems.append(legendItem);
      const positions = new Float32Array(samples.length * 3);
      for (const [pointIndex, point] of samples.entries()) {
        positions[pointIndex * 3] = Number(point.x) || 0;
        positions[pointIndex * 3 + 1] = Number(point.y) || 0;
        positions[pointIndex * 3 + 2] = 0.07;
      }
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
      geometry.setDrawRange(0, 1);
      const line = new THREE.Line(
        geometry,
        new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.95 })
      );
      line.frustumCulled = false;
      const marker = new THREE.Mesh(
        new THREE.SphereGeometry(0.13, 12, 8),
        new THREE.MeshBasicMaterial({ color })
      );
      marker.position.set(positions[0], positions[1], 0.12);
      comparisonGroup.add(line, marker);
      trajectories.push({ trajectory, samples, line, marker });
    }
    if (!trajectories.length) {
      comparisonReadout.textContent = "none";
      comparisonLegend.hidden = true;
      return;
    }
    comparisonGroup.visible = true;
    comparisonPlayback = {
      message,
      trajectories,
      startedAt: performance.now(),
      speed: 1.0,
      finished: false,
    };
    comparisonReadout.textContent = comparisonMetricsText(message, " · playing");
  }

  function interpolateComparisonSample(samples, elapsed) {
    if (samples.length === 1) return samples[0];
    const firstTime = Number(samples[0].time);
    const hasTiming = Number.isFinite(firstTime) && Number.isFinite(Number(samples[samples.length - 1].time));
    const targetTime = hasTiming ? elapsed : elapsed * 10;
    let high = samples.length - 1;
    let low = 0;
    while (low < high) {
      const middle = Math.floor((low + high) / 2);
      const sampleTime = hasTiming ? Number(samples[middle].time) : middle / 10;
      if (sampleTime < targetTime) low = middle + 1;
      else high = middle;
    }
    const nextIndex = Math.min(low, samples.length - 1);
    const previousIndex = Math.max(0, nextIndex - 1);
    const previous = samples[previousIndex];
    const next = samples[nextIndex];
    const previousTime = hasTiming ? Number(previous.time) : previousIndex / 10;
    const nextTime = hasTiming ? Number(next.time) : nextIndex / 10;
    const amount = nextTime > previousTime
      ? Math.max(0, Math.min(1, (targetTime - previousTime) / (nextTime - previousTime)))
      : 1;
    return {
      x: Number(previous.x) + (Number(next.x) - Number(previous.x)) * amount,
      y: Number(previous.y) + (Number(next.y) - Number(previous.y)) * amount,
    };
  }

  function updateComparisonPlayback() {
    if (!comparisonPlayback || !comparisonView) return;
    const elapsed = (performance.now() - comparisonPlayback.startedAt) / 1000 * comparisonPlayback.speed;
    let finished = true;
    for (const playback of comparisonPlayback.trajectories) {
      const samples = playback.samples;
      const lastSample = samples[samples.length - 1];
      const endTime = Number.isFinite(Number(lastSample.time))
        ? Number(lastSample.time) : (samples.length - 1) / 10;
      const currentTime = Math.min(elapsed, endTime);
      let visibleCount = 1;
      for (let index = 1; index < samples.length; index += 1) {
        const sampleTime = Number.isFinite(Number(samples[index].time))
          ? Number(samples[index].time) : index / 10;
        if (sampleTime <= currentTime) visibleCount = index + 1;
        else break;
      }
      playback.line.geometry.setDrawRange(0, visibleCount);
      const position = interpolateComparisonSample(samples, currentTime);
      playback.marker.position.set(position.x, position.y, 0.12);
      if (elapsed < endTime) finished = false;
    }
    if (finished && !comparisonPlayback.finished) {
      comparisonPlayback.finished = true;
      comparisonReadout.textContent = comparisonMetricsText(comparisonPlayback.message, " · complete");
    }
  }

  function forceCheckbox(name) {
    if (name === "gravity") return document.querySelector('[data-force="gravity"]');
    if (name.startsWith("normal_")) return document.querySelector('[data-force="normal_front"]');
    if (name.startsWith("suspension_")) return document.querySelector('[data-force="normal_front"]');
    if (name.startsWith("tire_")) return document.querySelector('[data-force="tire_front_lateral"]');
    if (name.startsWith("rolling_")) return document.querySelector('[data-force="rolling_resistance"]');
    return document.querySelector(`[data-force="${name}"]`);
  }

  function updateForceArrows(snapshot) {
    const breakdown = snapshot.diagnostics && snapshot.diagnostics.force_breakdown;
    if (!breakdown || !car) return;
    const vehicle = snapshot.vehicle || {};
    // Force arrows are a visualization in metres, while the dynamics report
    // forces in newtons. Keep the display proportional to force but bounded by
    // the vehicle's physical scale. The bound prevents a large normal/grip
    // force from looking like the wheel has an impossible steering radius.
    const bodyLength = Math.max(Number(vehicle.body_length) || 0.6, 0.2);
    const wheelbase = Math.max(Number(vehicle.wheelbase) || 0.33, 0.1);
    const maxArrowLength = Math.max(0.55, Math.min(bodyLength * 1.55, wheelbase * 1.9));
    const minimumArrowLength = Math.max(0.16, bodyLength * 0.24);
    const metresPerNewton = Math.max(0.018, bodyLength * 0.035);
    const colors = {
      gravity: 0xa855f7, normal_front: 0x22d3ee, normal_rear: 0x22d3ee,
      tire_front_lateral: 0xff3b81, tire_rear_lateral: 0xff3b81,
      tire_rear_longitudinal: 0xff3b81, rolling_resistance: 0xfb923c,
      aerodynamic_drag: 0x38bdf8,
    };
    const colorForForce = (name) => {
      if (name.startsWith("tire_")) return 0xff3b81;
      if (name.startsWith("suspension_") || name.startsWith("normal_")) return 0x22d3ee;
      if (name.startsWith("rolling_")) return 0xfb923c;
      if (name.startsWith("aerodynamic_")) return 0x38bdf8;
      if (name === "gravity") return 0xa855f7;
      return colors[name] || 0xffd84d;
    };
    for (const [name, component] of Object.entries(breakdown)) {
      if (name === "total_external") continue;
      const checkbox = forceCheckbox(name);
      const vector = new THREE.Vector3(...(component.force_body || [0, 0, 0]));
      const magnitude = vector.length();
      if (!checkbox || !checkbox.checked || !component.enabled || magnitude < 1e-5) {
        if (car.forceArrows && car.forceArrows[name]) car.forceArrows[name].visible = false;
        continue;
      }
      let arrow = car.forceArrows && car.forceArrows[name];
      if (!arrow) {
        const arrowColor = colorForForce(name);
        arrow = new THREE.ArrowHelper(
          new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.1,
          arrowColor, 0.08, 0.045,
        );
        arrow.shaft = new THREE.Mesh(
          new THREE.CylinderGeometry(0.5, 0.5, 1, 8),
          new THREE.MeshBasicMaterial({ color: arrowColor, toneMapped: false }),
        );
        arrow.add(arrow.shaft);
        car.forceGroup.add(arrow);
        car.forceArrows = car.forceArrows || {};
        car.forceArrows[name] = arrow;
      }
      arrow.visible = true;
      arrow.position.set(...(component.application_point_body || [0, 0, 0.15]));
      // The plant already applies the per-wheel friction-circle limit. Keep a
      // defensive clamp here as well so a transient floating-point overshoot
      // cannot be rendered as a physically impossible tire force.
      const forceLimit = Number(component.force_limit);
      const displayMagnitude = Number.isFinite(forceLimit)
        ? Math.min(magnitude, Math.max(0, forceLimit))
        : magnitude;
      if (magnitude > 1e-9) vector.multiplyScalar(displayMagnitude / magnitude);
      arrow.setDirection(vector.normalize());
      const length = Math.min(
        maxArrowLength,
        Math.max(minimumArrowLength, 0.08 + displayMagnitude * metresPerNewton),
      );
      const headLength = Math.min(0.16, Math.max(0.07, length * 0.18));
      const headWidth = Math.min(0.10, Math.max(0.045, length * 0.11));
      const stemLength = Math.max(0.0001, length - headLength);
      const shaftDiameter = Math.max(0.028, bodyLength * 0.045);
      arrow.setLength(
        length,
        headLength,
        headWidth,
      );
      arrow.shaft.position.y = stemLength * 0.5;
      arrow.shaft.scale.set(shaftDiameter, stemLength, shaftDiameter);
    }
  }

  function updateReadouts(snapshot) {
    const state = snapshot.state;
    const diagnostics = snapshot.diagnostics || {};
    const total = diagnostics.force_breakdown && diagnostics.force_breakdown.total_external;
    const force = total ? new THREE.Vector3(...total.force_body).length() : 0;
    const active = Object.entries(diagnostics.force_breakdown || {})
      .filter(([name, value]) => name !== "total_external" && value.enabled && forceCheckbox(name)?.checked)
      .map(([name]) => name.replaceAll("_", " "));
    statusReadout.textContent = snapshot.playing ? "simulation running" : "simulation paused";
    tReadout.textContent = `${Number(snapshot.time).toFixed(2)} s`;
    posReadout.textContent = `${Number(state.x).toFixed(2)}, ${Number(state.y).toFixed(2)}, ${Number(state.z || 0).toFixed(2)}`;
    motionReadout.textContent = `${Number(state.vx).toFixed(2)} m/s / ${Number(state.yaw_rate).toFixed(2)} rad/s`;
    forceReadout.textContent = `${force.toFixed(2)} N / ${(Number(diagnostics.friction_utilization) * 100).toFixed(0)}%`;
    const wind = snapshot.wind || {};
    activeWindSpeed = Number(wind.speed) || 0.0;
    windReadout.textContent = `${Number(wind.speed || 0).toFixed(1)} m/s @ ${Number(wind.direction_deg || 0).toFixed(0)}°`;
    breakdownReadout.textContent = active.length ? active.join(", ") : "none";
  }

  function updateWindVisual(now = 0) {
    const wind = latestSnapshot && latestSnapshot.wind ? latestSnapshot.wind : {};
    const speed = Number(wind.speed) || 0.0;
    if (!car || comparisonView || speed < 0.2) {
      windGroup.visible = false;
      return;
    }
    windGroup.visible = true;
    const intensity = Math.max(0, Math.min(1, speed / 32.0));
    const direction = THREE.MathUtils.degToRad(Number(wind.direction_deg) || 0.0);
    const flow = new THREE.Vector3(Math.cos(direction), Math.sin(direction), 0);
    const cross = new THREE.Vector3(-flow.y, flow.x, 0);
    const anchor = car.root.position.clone();
    const wrapDistance = 4.5 + 4.0 * intensity;
    const streamRate = 0.35 + speed * 0.12;
    const streakLength = 0.28 + 1.05 * intensity;
    const headLength = Math.min(streakLength * 0.34, 0.42);
    const headWidth = 0.07 + 0.08 * intensity;
    const activeCount = Math.max(6, Math.round(8 + intensity * 28));
    const elapsed = Number(now) / 1000;
    for (const [index, streak] of windStreaks.entries()) {
      const line = streak.line;
      if (index >= activeCount) {
        line.visible = false;
        continue;
      }
      line.visible = true;
      const along = ((elapsed * streamRate + streak.phase) % 1 - 0.5) * wrapDistance;
      const center = anchor.clone()
        .addScaledVector(cross, streak.lateral * 5.0)
        .addScaledVector(flow, along)
        .setZ(anchor.z + streak.vertical);
      const start = center.clone().addScaledVector(flow, -streakLength * 0.5);
      const end = center.clone().addScaledVector(flow, streakLength * 0.5);
      const headBase = end.clone().addScaledVector(flow, -headLength);
      const headLeft = headBase.clone().addScaledVector(cross, headWidth);
      const headRight = headBase.clone().addScaledVector(cross, -headWidth);
      const positions = line.geometry.attributes.position.array;
      positions.set([
        start.x, start.y, start.z, end.x, end.y, end.z,
        end.x, end.y, end.z, headLeft.x, headLeft.y, headLeft.z,
        end.x, end.y, end.z, headRight.x, headRight.y, headRight.z,
      ]);
      line.geometry.attributes.position.needsUpdate = true;
      line.material.opacity = 0.24 + 0.52 * intensity;
    }
  }


  function animate(now) {
    resize();
    updateComparisonPlayback();
    updateAutoCamera();
    updateWindVisual(now);
    renderer.render(scene, camera);
    requestAnimationFrame(animate);
  }

  loadRaceTracks();
  populateComparisonModels();
  connectSimulation();
  requestAnimationFrame(animate);
}());
