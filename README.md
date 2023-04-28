# Havok for the Web

This package contains an esm and umd version of the Havok physics library for the web.

## How to use

Import havok to your project:

```javascript
import HavokPhysics from "@babylonjs/havok";
```

And then initialize the physics engine (inside an async function, or in a modern browser on the main scope):

```javascript
const havokInterface = await HavokPhysics();
```

Note that `HavokPhysics` returns a promise with the initialized engine. You can, of course, use `then` instead of `await` if you prefer.

## Using the Babylon physics plugin with Havok

Havok is fully integrated in Babylon.js. To use it in a Babylon project, pass it then as the 2nd variable to the Havok physics plugin:

```javascript
import { PhysicsEngine, HavokPlugin } from "@babylonjs/core/Physics";

const plugin = new HavokPlugin(undefined /* or the value that fits your usecase */, havokInterface);
scene.enablePhysics(undefined /* or the value that fits your usecase, for example: new Vector3(0, -9.81, 0) */, plugin);
```

And you are good to go!

## Questions

The babylon team will be happy to answer any questions! Please contact us at https://forum.babylonjs.com
