# Physics-Based Ackermann Vehicle Digital Twin

An interactive browser simulation for visualizing a four-wheel vehicle moving
through F1TENTH race tracks. The application separates the visual vehicle mesh
from the physical vehicle state: changing a model changes the rendered car and
its selected physical preset, while the Python simulation remains the source
of truth for forces and motion.

## How the simulation was created

The project was built in layers:

1. F1TENTH centerline CSV files were loaded and reconstructed into metric,
   closed race paths with headings, widths, curvature, and a benchmark speed
   profile.
2. A path-following controller was added using nearest-point matching, a
   lookahead target, pure-pursuit steering, heading/lateral-error correction,
   and preview braking before corners.
3. The original dynamic bicycle equations were extended into a four-wheel
   six-degree-of-freedom plant. The chassis state includes position, velocity,
   roll, pitch, yaw, angular rates, and individual wheel angular velocities.
4. Each tire calculates longitudinal slip and slip angle. Longitudinal and
   lateral forces are combined and limited by the friction circle:

   ```text
   sqrt(F_longitudinal² + F_lateral²) <= μ * normal_load
   ```

5. Suspension compression, normal loads, gravity, rolling resistance,
   aerodynamic drag, relative-air wind forces, terrain friction, and deterministic
   terrain height
   variation contribute to the vehicle forces and moments.
6. Three.js renders the physical state in the browser. GLB models are scaled
   to the active physical dimensions, oriented into the simulation coordinate
   frame, and given wheel steering/spin pivots. The browser sends commands, but
   the Python server advances the physical state.

## Included features

- Six-DOF vehicle physics with tire-force saturation.
- Editable mass, wheelbase, inertia, tire, suspension, drag, wind, rolling-resistance,
  center-of-mass, wheel, and speed parameters.
- F1TENTH Austria, Barcelona, Great Britain, and Monaco race paths.
- Automatic path following and changing corner-speed behavior.
- Terrain choices: dry/wet asphalt, gravel, grass, mud, and ice.
- Wind profiles: calm, light breeze, moderate wind, strong wind, storm, and hurricane.
- Third-person chase, top-down, and trackside camera views.
- Model comparison and terrain comparison modes with animated colored paths and
  legends.
- Force arrows for gravity, normal loads, tires, drive, rolling resistance, and
  aerodynamic drag.
- Local imported GLB vehicles and local Three.js dependencies.

## Requirements

- Python 3.10 or newer
- NumPy
- A modern browser with WebGL support

Install the Python dependency from this directory:

```bash
python3 -m pip install -r requirements.txt
```

## Launching the simulation

From the repository root:

```bash
./run.sh
```

Then open:

```text
http://127.0.0.1:8080
```

Alternatively, launch the server directly:

```bash
python3 digital_twin_viewer/sim_server.py --host 127.0.0.1 --port 8080
```

Do not open `index.html` directly with `file://`. The viewer needs the local
HTTP server for GLB assets, the track API, and the WebSocket simulation
connection.

## Operating the viewer

The control panel is scrollable. The main controls are:

- `Pause` / `Play`: pause or resume the physical simulation.
- `Reset`: pause the simulation, clear motion commands, and place the active
  vehicle at the start of the selected race path.
- `F1TENTH track`: select Austria, Barcelona, Great Britain, or Monaco.
- `Load track`: rebuild the selected track and reset the vehicle.
- `Camera view`: choose chase, top-down, or trackside presentation.
- `Terrain`: change surface appearance, friction, rolling resistance, and
  terrain height variation.
- `Vehicle & wind profiles`: select one of the imported vehicles or the Race,
  Sedan, SUV, and Van presets, and choose a named wind condition. Changing the
  vehicle or wind profile resets and pauses the run.
- Editable parameter fields: change vehicle properties, then leave the field or
  press Enter to send the new values to the server.
- `Show forces`: toggle individual physical force vectors.

Wind is modeled from the vehicle's relative air velocity. The wind profile sets
speed and direction, while the editable aerodynamic fields expose air density and
drag coefficients; crosswind forces and the resulting aerodynamic yaw moment are
included in the six-DOF plant. The viewer also renders animated wind streaks around
the vehicle. Their density,
length, opacity, and flow rate increase with the selected wind intensity.

### Compare models

Choose two models under `Compare models` and press `Compare models`. The viewer
switches to a complete top-down track view, simulates both vehicles on the same
track and terrain, and draws their paths as colored lines with moving markers.
The legend identifies each line.

### Compare terrains

Select two different terrains under `Compare terrains` and press
`Compare terrains`. The selected vehicle is simulated on both surfaces. The
result shows how friction, rolling resistance, and terrain height alter speed
and path tracking. The legend identifies each terrain line.

## Architecture

```text
Browser / Three.js
  - controls and editable parameters
  - GLB vehicle rendering
  - camera and path visualization
  - WebSocket commands
          |
          v
Python simulation server
  - track catalog and path controller
  - terrain presets
  - physical state and time integration
          |
          v
Six-DOF dynamics plant
  - four tire contacts
  - slip ratio and slip angle
  - friction-circle saturation
  - suspension and normal loads
  - chassis forces and moments
```

Core files:

- `digital_twin_viewer/sim_server.py`: HTTP/WebSocket server, tracks, terrain,
  path following, and comparison simulation.
- `GRL_SNAM_merged_digital_twin/experiments/corl_ackermann/environment_dynamics.py`:
  dynamic-bicycle configuration and equations.
- `GRL_SNAM_merged_digital_twin/experiments/corl_ackermann/high_fidelity_dynamics.py`:
  four-wheel six-DOF plant and force diagnostics.
- `digital_twin_viewer/app.js`: Three.js scene, controls, model loading, camera,
  path display, and comparison playback.
- `f1tenth_benchmarks/maps/*_centerline.csv`: included metric race centerlines.

## Asset licensing

See `digital_twin_viewer/assets/imported_cars/ATTRIBUTION.md` and
`digital_twin_viewer/assets/kenney_car/License.txt` for the included model
licenses.
