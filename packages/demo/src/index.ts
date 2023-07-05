import { Engine } from "@babylonjs/core/Engines/engine";
import { createScene } from "./createScene";

export const babylonInit = async (): Promise<void> => {
    // Get the canvas element
    const canvas = document.getElementById("renderCanvas") as HTMLCanvasElement;
    // Generate the BABYLON 3D engine
    let engine = new Engine(canvas, true);
    // Create the scene
    const scene = await createScene({ engine, canvas });

    // JUST FOR TESTING. Not needed for anything else
    (window as any).scene = scene;

    // Register a render loop to repeatedly render the scene
    engine.runRenderLoop(function () {
        scene.render();
    });

    // Watch for browser/canvas resize events
    window.addEventListener("resize", function () {
        engine.resize();
    });
};

babylonInit();