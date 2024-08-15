function createShapes(hp) {
    let shapes = [];
    [_, sphereShape] = hp.HP_Shape_CreateSphere([0,0,0], 1.0);
    shapes.push(sphereShape);
    [_, boxShape] = hp.HP_Shape_CreateBox([0,0,0], [0,0,0,1], [1,1,1]);
    shapes.push(boxShape);
    {
        const numCvxVerts = 16;
        const radius = 2.0;
        let vertBuffer = hp._malloc(numCvxVerts * 3 * 4);
        let verts = new Float32Array(hp.HEAPU8.buffer, vertBuffer, numCvxVerts * 3);
        for (let i = 0; i < numCvxVerts; i++) {
            verts[i * 3 + 0] = Math.random() * radius;
            verts[i * 3 + 1] = Math.random() * radius;
            verts[i * 3 + 2] = Math.random() * radius;
        }
        [_, cvxShape] = hp.HP_Shape_CreateConvexHull(vertBuffer, numCvxVerts);
        hp._free(vertBuffer);
    }

    return [sphereShape, boxShape, cvxShape];
}

load("./HavokPhysics.js");
HavokPhysics().then(
    (hp) => {
        [_, world] = hp.HP_World_Create();

        [_, groundShape] = hp.HP_Shape_CreateBox([0,0,0], [0,0,0,1], [100, 1, 100]);
        [_, groundBody] = hp.HP_Body_Create();
        hp.HP_Body_SetShape(groundBody, groundShape);
        hp.HP_World_AddBody(world, groundBody, true);

        let shapes = createShapes(hp);

        const numBodies = 1000;
        let testBody = null;
        for (let i = 0; i < numBodies; i++)
        {
            let [_, body] = hp.HP_Body_Create();
            hp.HP_Body_SetShape(body, shapes[Math.floor(Math.random() * shapes.length)]);
            hp.HP_Body_SetMotionType(body, hp.MotionType.DYNAMIC);
            hp.HP_Body_SetQTransform(body, [[Math.random() * 5,10 + Math.random() * 5, Math.random() * 5], [0,0,0,1]]);
            hp.HP_World_AddBody(world, body, false);
            if (i == 0) {
                testBody = body;
            }
        }

        const deltaTime = 1.0 / 60.0;
        for(let i = 0; i < 600; i++) {
            if (i % 60 == 0) {
                [_, [translation, orient]] = hp.HP_Body_GetQTransform(testBody);
                [_, linVel] = hp.HP_Body_GetLinearVelocity(testBody);
                console.log("Time", i * deltaTime, "Pos", translation, "LinVel", linVel);
            }

            hp.HP_World_Step(world, deltaTime);
        }
    }
);
