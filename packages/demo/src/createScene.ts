import { Scene } from '@babylonjs/core/scene';
import { Engine } from '@babylonjs/core/Engines/engine';
import { Vector3 } from '@babylonjs/core/Maths/math.vector';
import { FreeCamera } from '@babylonjs/core/Cameras/freeCamera';
import { HemisphericLight } from '@babylonjs/core/Lights/hemisphericLight';
import { MeshBuilder } from '@babylonjs/core/Meshes/meshBuilder';
import HavokPhysics from '@babylonjs/havok';
import { PhysicsShapeType } from '@babylonjs/core/Physics/v2/IPhysicsEnginePlugin';
import { PhysicsAggregate } from '@babylonjs/core/Physics/v2/physicsAggregate';
import { HavokPlugin } from '@babylonjs/core/Physics/v2/Plugins/havokPlugin';
import "@babylonjs/core/Physics/v2/physicsEngineComponent";
import "@babylonjs/core/Materials/standardMaterial";

async function getInitializedHavok() {
    return await HavokPhysics();
}

export async function createScene({ engine, canvas }: { engine: Engine; canvas: HTMLCanvasElement; }): Promise<Scene> {

    // This creates a basic Babylon Scene object (non-mesh)
    var scene = new Scene(engine);

    // This creates and positions a free camera (non-mesh)
    var camera = new FreeCamera("camera1", new Vector3(0, 5, -10), scene);

    // This targets the camera to scene origin
    camera.setTarget(Vector3.Zero());

    // This attaches the camera to the canvas
    camera.attachControl(canvas, true);

    // This creates a light, aiming 0,1,0 - to the sky (non-mesh)
    var light = new HemisphericLight("light", new Vector3(0, 1, 0), scene);

    // Default intensity is 1. Let's dim the light a small amount
    light.intensity = 0.7;

    // Our built-in 'sphere' shape.
    var sphere = MeshBuilder.CreateSphere("sphere", { diameter: 2, segments: 32 }, scene);

    // Move the sphere upward at 4 units
    sphere.position.y = 4;

    // Our built-in 'ground' shape.
    var ground = MeshBuilder.CreateGround("ground", { width: 10, height: 10 }, scene);
    const havok = await getInitializedHavok();
    // initialize plugin
    var hk = new HavokPlugin(undefined, havok);
    // enable physics in the scene with a gravity
    scene.enablePhysics(new Vector3(0, -9.8, 0), hk);

    // Create a sphere shape and the associated body. Size will be determined automatically.
    var sphereAggregate = new PhysicsAggregate(sphere, PhysicsShapeType.SPHERE, { mass: 1, restitution: 0.75 }, scene);

    // Create a static box shape.
    var groundAggregate = new PhysicsAggregate(ground, PhysicsShapeType.BOX, { mass: 0 }, scene);

    return scene;

}