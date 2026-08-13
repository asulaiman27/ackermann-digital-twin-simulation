import * as ThreeNamespace from "./three.module.js";
import { GLTFLoader } from "./GLTFLoader.js";

// app.js is intentionally kept as a browser-global script. Expose the pinned
// Three.js namespace and loader before importing it.
window.THREE = { ...ThreeNamespace, GLTFLoader };
await import("./app.js");
