/// <reference types="emscripten" />
export type Vector3 = [/* x */ number, /* y */ number, /* z */ number];
export type Quaternion = [/* x */ number, /* y */ number, /* z */ number, /* w */ number];
export type Rotation = [/* column 0 */ Vector3, /* column 1 */ Vector3, /* column 2 */ Vector3];
export type QTransform = [/* translation */ Vector3, /* rotation */ Quaternion];
export type QSTransform = [/* translation */ Vector3, /* rotation */ Quaternion, /* scale */ Vector3];
export type Transform = [/* translation */ Vector3, /* rotation */ Rotation];
export type Aabb = [/* min */ Vector3, /* max */ Vector3];

export type HP_BodyId = [bigint];
export type HP_ShapeId = [bigint];
export type HP_ConstraintId = [bigint];
export type HP_WorldId = [bigint];
export type HP_CollectorId = [bigint];
export type HP_DebugGeometryId = [bigint];

export type ShapePathIterator = [/*shapeId*/ bigint, /*pathData*/ bigint];

export enum Result {
    RESULT_OK,
    RESULT_FAIL,
    RESULT_INVALIDHANDLE,
    RESULT_INVALIDARGS,
    RESULT_NOTIMPLEMENTED
}

export enum ShapeType {
    COLLIDER,
    CONTAINER
}

export enum MotionType {
    STATIC,
    KINEMATIC,
    DYNAMIC
}

export enum EventType {
    COLLISION_STARTED,
    COLLISION_CONTINUED,
    COLLISION_FINISHED
}

export enum ConstraintMotorType {
    NONE,
    VELOCITY,
    POSITION
}

export enum ConstraintAxisLimitMode {
    FREE,
    LIMITED,
    LOCKED
}

export enum ConstraintAxis {
    LINEAR_X,
    LINEAR_Y,
    LINEAR_Z,
    ANGULAR_X,
    ANGULAR_Y,
    ANGULAR_Z,
    LINEAR_DISTANCE
}

export type MassProperties = [
    /* center of mass */ Vector3,
    /* Mass */ number,
    /* Inertia for mass of 1*/ Vector3,
    /* Inertia Orientation */ Quaternion
];

export enum MaterialCombine {
    GEOMETRIC_MEAN,
    MINIMUM,
    MAXIMUM,
    ARITHMETIC_MEAN,
    MULTIPLY
}

export type PhysicsMaterial = [
    /* static friction */ number,
    /* dynamic friction */ number,
    /* restitution */ number,
    /* friction combine mode */ MaterialCombine,
    /* restitution combine mode */ MaterialCombine
];

export type FilterInfo = [
    /* Membership mask */ number,
    /* Collision mask */ number
];

export type ContactPoint = [
    /* Body ID */ HP_BodyId,
    /* Collider ID */ HP_ShapeId,
    /* Shape hierarchy */ ShapePathIterator,
    /* Position */ Vector3,
    /* Normal */ Vector3,
    /* Triangle index */ number
];

export type RayCastInput = [
    /* Start */ Vector3,
    /* End */ Vector3,
    /* Collision filter info */ FilterInfo
];

export type RayCastResult = [
    /* Fraction */ number,
    /* Contact point */ ContactPoint
];

export type BodyMovedEvent = [
    /* Type */ EventType,
    /* Body ID */ HP_BodyId,
    /* World from body */ QTransform
];

export type CollisionEvent = [
    /* Type */ EventType,
    /* Contact on body A */ ContactPoint,
    /* Contact on body B */ ContactPoint,
    /* Impulse applied */ number
];
export type DebugGeometryInfo = [
    /* Address of vertex (float3) buffer in plugin */ number,
    /* Number of vertices in the buffer */ number,
    /* Address of triangle (int, int, int) buffer in plugin */ number,
    /* Number of triangle in the buffer */ number
];

export type ObjectStatistics = [
    /* NumBodies */ number,
    /* NumShapes */ number,
    /* NumConstraints */ number,
    /* NumDebugGeometries */ number,
    /* NumWorlds */ number,
    /* NumQueryCollectors */ number
];

export interface HavokPhysicsWithBindings extends EmscriptenModule {
    /* Return statistics on the number of allocated objects in the plugin */
    HP_GetStatistics(): [Result, ObjectStatistics];
    /** Configures the collision filter to enable/disable a particular pair of bodies from colliding. */
    HP_SetBodyCollisionEnabled(bodyA : HP_BodyId, bodyB : HP_BodyId, isEnabled : number): Result;
    /** Gets the configuration of the collision filter for the two specified bodies. */
    HP_GetBodyCollisionEnabled(bodyA : HP_BodyId, bodyB : HP_BodyId): [Result, number];
    /** Creates geometry representing a sphere. */ 
    HP_Shape_CreateSphere(center : Vector3, radius : number): [Result, HP_ShapeId];
    /** Creates a geometry representing a capsule. */ 
    HP_Shape_CreateCapsule(pointA : Vector3, pointB : Vector3, radius : number): [Result, HP_ShapeId];
    /** Creates a geometry representing a cylinder. */ 
    HP_Shape_CreateCylinder(pointA : Vector3, pointB : Vector3, radius : number): [Result, HP_ShapeId];
    /** Creates a geometry representing a box.
    * @param center - Position of the box center (in shape space)
    * @param rotation - Orientation of the box (in shape space)
    * @param extents - total size of the box */
    HP_Shape_CreateBox(center : Vector3, rotation : Quaternion, extents : Vector3): [Result, HP_ShapeId];
    /** Creates a geometry which encloses all the `vertices`.
    * Note, vertices need to be allocated within the WASM memory using `_malloc` and should refer to a buffer populated with `Vector`. */ 
    HP_Shape_CreateConvexHull(vertices : number, numVertices : number): [Result, HP_ShapeId];
    /** Creates a geometry representing the surface of a mesh.
    * Note, like CreateConvexHull, `vertices` should be a buffer of `Vector`, allocated using `_malloc`.
    * Similarly, triangles should be triples of 32-bit integers which index into `vertices`. */ 
    HP_Shape_CreateMesh(vertices : number, numVertices : number, triangles : number, numTriangles : number): [Result, HP_ShapeId];
    /** Creates a geometry representing a height map.
     * `heights` should be a buffer of floats, of size (numXSamples * numZSamples), describing heights at (x,z) of
     * [(0,0), (1,0), ... (numXSamples-1, 0), (0, 1), (1, 1) ... (numXSamples-1, 1) ... (numXSamples-1, numZSamples-)]
     * `scale` is a vector, whose X and Z components convert from integer space to shape space, while the Y coordinate supplies a scaling factor for the height. */
    HP_Shape_CreateHeightField(numXSamples : number, numZSamples : number, scale : Vector3, heights : number) : [Result, HP_ShapeId];
    /** Sets the collision info for the shape to the information in `filterInfo`. This can prevent collisions between shapes and queries, depending on how you have configured the filter. */ 
    HP_Shape_SetFilterInfo(shapeId : HP_ShapeId, filterInfo: FilterInfo): Result;
    /** Get the collision filter info for a shape. */ 
    HP_Shape_GetFilterInfo(shapeId : HP_ShapeId): [Result, FilterInfo];
    /** Sets the material of the shape to the provided material. */ 
    HP_Shape_SetMaterial(shapeId : HP_ShapeId, materialId : PhysicsMaterial): Result;
    /** Get the material associated with the shape. */ 
    HP_Shape_GetMaterial(shapeId : HP_ShapeId): [Result, PhysicsMaterial];
    /** Set the density of the shape. Used when calling `HP_Shape_BuildMassProperties()`. */ 
    HP_Shape_SetDensity(shapeId : HP_ShapeId, density : number): Result;
    /** Get the density of the shape. */ 
    HP_Shape_GetDensity(shapeId : HP_ShapeId): [Result, number];
    /** Creates a "container" shape - this shape does not have any inherent geometry, but it can contain other shapes. */ 
    HP_Shape_CreateContainer(): [Result, HP_ShapeId];
    /** Adds the `newChild` to the container `container` at the transform `containerFromChild`. */ 
    HP_Shape_AddChild(container : HP_ShapeId, newChild : HP_ShapeId, containerFromChild : QSTransform): Result;
    /** Removes the child at index `childIndex` inside `container`. */ 
    HP_Shape_RemoveChild(container : HP_ShapeId, childIndex : number): Result;
    /** Get the number of children of the container. */ 
    HP_Shape_GetNumChildren(container : HP_ShapeId): [Result, number];
    /** Changes the child shape at `index` of `container` to `newChild`. The transform of the child is unchanged. */ 
    HP_Shape_SetChildShape(container : HP_ShapeId, index : number, newChild : HP_ShapeId): Result;
    /** Get the shape at index `index` inside `container`. */ 
    HP_Shape_GetChildShape(container : HP_ShapeId, index : number): [Result, HP_ShapeId];
    /** Change the transform of the child at index `index` of `container`. */ 
    HP_Shape_SetChildQTransform(container : HP_ShapeId, index : number, containerFromChild : QSTransform): Result;
    /** Get the transform of the child at index `index` of `container` (in the space of the container.) */ 
    HP_Shape_GetChildQSTransform(container : HP_ShapeId, index : number): [Result, QSTransform];
    /** Get the type of the shape - CONTAINER or COLLIDER. */ 
    HP_Shape_GetType(shape : HP_ShapeId): [Result, ShapeType];
    /** Sets an arbitrary user data on the shape. Not used by the engine. */ 
    //HP_Shape_SetUserData(shape : HP_ShapeId, userdata : number): Result;
    /** Gets the user data associated with the shape. */ 
    //HP_Shape_GetUserData(shape : HP_ShapeId): [Result, number];
    /** Release a shape, freeing memory if it is unused. */ 
    HP_Shape_Release(shape : HP_ShapeId): Result;
    /** Calculate the axis-aligned bounding box of the shape (in shape space) */ 
    HP_Shape_GetBoundingBox(shape : HP_ShapeId): [Result, Aabb];
    /** Calculates the mass properties of the shape. */ 
    HP_Shape_BuildMassProperties(shape : HP_ShapeId): [Result, MassProperties];
    /** Allows descending a hierarchy of shape containers, advancing `curItem` to the next entry. */ 
    HP_Shape_PathIterator_GetNext(curItem : ShapePathIterator): [Result, ShapePathIterator, number];

    /** Generates a visualization of a shape's geometry, suitable for debugging. */
    HP_Shape_CreateDebugDisplayGeometry( shape : HP_ShapeId ) : [Result, HP_DebugGeometryId];
    /** Retrieves the vertex and triangle information for a debug geometry ID. */
    HP_DebugGeometry_GetInfo( geometry : HP_DebugGeometryId ) : [Result, DebugGeometryInfo];
    /** Release the reference to the debug geometry, freeing memory. */
    HP_DebugGeometry_Release( geometry : HP_DebugGeometryId ) : Result;
    /** Allocates a new body. */ 
    HP_Body_Create(): [Result, HP_BodyId];
    /** Releases a body, potentially freeing the memory. Will not remove from the world if body is in use. */ 
    HP_Body_Release(bodyId : HP_BodyId): Result;
    /** Sets body to use a particular shape.
    * A body can only have a single shape; multiple shapes should be wrapped in a container shape. */ 
    HP_Body_SetShape(bodyId : HP_BodyId, shapeId : HP_ShapeId): Result;
    /** Get the shape in use by a body. */ 
    HP_Body_GetShape(bodyId : HP_BodyId): [Result, HP_ShapeId];
    /** Set the body to behave according to the motion type. */ 
    HP_Body_SetMotionType(bodyId : HP_BodyId, motionType : MotionType): Result;
    /** Get the current motion type of the body. */ 
    HP_Body_GetMotionType(bodyId : HP_BodyId): [Result, MotionType];
    /** Configure a body to raise events, based on eventMask. Bodies will not raise events by default. The event mask should be the integer value of the EventType enum for all the events you wish to opt into, ORed together. */ 
    HP_Body_SetEventMask(bodyId : HP_BodyId, eventMask : number): Result;
    /** Get the event mask of a body. */ 
    HP_Body_GetEventMask(bodyId : HP_BodyId): [Result, number];
    /** Configures the body to use the supplied mass properties. */ 
    HP_Body_SetMassProperties(bodyId : HP_BodyId, massProps : MassProperties): Result;
    /** Get the mass properties currently used by the body. */ 
    HP_Body_GetMassProperties(bodyId : HP_BodyId): [Result, MassProperties];
    /** Sets the linear damping of a body. This will reduce the linear velocity of the body by some fraction every step, even when the body is not in collision. */ 
    HP_Body_SetLinearDamping(bodyId : HP_BodyId, damping : number): Result;
    /** Get the linear damping of a body. */ 
    HP_Body_GetLinearDamping(bodyId : HP_BodyId): [Result, number];
    /** Sets the angular damping of a body. This will reduce the angular velocity of the body by some fraction every step, event when the body is not in collision. */ 
    HP_Body_SetAngularDamping(bodyId : HP_BodyId, damping : number): Result;
    /** Get the angular damping of a body. */ 
    HP_Body_GetAngularDamping(bodyId : HP_BodyId): [Result, number];
    /** Set the gravity factor of the body. This will scale the effect of gravity on the body during the simulation step. */ 
    HP_Body_SetGravityFactor(bodyId : HP_BodyId, factor : number): Result;
    /** Get the gravity factor of a body. */ 
    HP_Body_GetGravityFactor(bodyId : HP_BodyId): [Result, number];
    /** Sets an arbitrary user data on the body. Unused by the engine. */ 
    //HP_Body_SetUserData(bodyId : HP_BodyId, userData : number): Result;
    /** Get the user data associated with a body. */ 
    //HP_Body_GetUserData(bodyId : HP_BodyId): [Result, number];
    /* Return the offet of the memory containing the body's transform from the
     * world's body buffer. This is only valid if the body is in the world. */
    HP_Body_GetWorldTransformOffset( bodyId: HP_BodyId ): [Result, number];
    /** Sets the position of the body (in world space.) */ 
    HP_Body_SetPosition(bodyId : HP_BodyId, position : Vector3): Result;
    /** Get the position of the body (in world space.) */ 
    HP_Body_GetPosition(bodyId : HP_BodyId): [Result, Vector3];
    /** Set the orientation of the body (in world space.) */ 
    HP_Body_SetOrientation(bodyId : HP_BodyId, orientation : Quaternion): Result;
    /** Get the orientation of the body (in world space.) */ 
    HP_Body_GetOrientation(bodyId : HP_BodyId): [Result, Quaternion];
    /** Set the transform of the body (in world space.) */ 
    HP_Body_SetQTransform(bodyId : HP_BodyId, transform : QTransform): Result;
    /** Get the transform of the body (in world space.) */ 
    HP_Body_GetQTransform(bodyId : HP_BodyId): [Result, QTransform];
    /** Set the transform of the body (in world space.) */ 
    HP_Body_SetTransform(bodyId : HP_BodyId, transform : Transform): Result;
    /** Get the transform of the body (in world space.) */ 
    HP_Body_GetTransform(bodyId : HP_BodyId): [Result, Transform];
     /** Set the linear velocity of a body. No effect on STATIC bodies. */ 
    HP_Body_SetLinearVelocity(bodyId : HP_BodyId, linVel : Vector3): Result;
    /** Get the linear velocity of a body. */ 
    HP_Body_GetLinearVelocity(bodyId : HP_BodyId): [Result, Vector3];
    /** Set the angular velocity of a body. No effect on STATIC bodies. */ 
    HP_Body_SetAngularVelocity(bodyId : HP_BodyId, angVel : Vector3): Result;
    /** Get the angular velocity of a body. */ 
    HP_Body_GetAngularVelocity(bodyId : HP_BodyId): [Result, Vector3];
    /** Change the body velocity such that next step, it would reach the target transform. DYNAMIC bodies can still be prevented from reaching that transform by collisions and constraints. */ 
    HP_Body_SetTargetQTransform(bodyId : HP_BodyId, transform : QTransform): Result;
    /** Apply the impulse `impulse` to the body at the position `location` in world space. */ 
    HP_Body_ApplyImpulse(bodyId : HP_BodyId, location : Vector3, impulse : Vector3): Result;
    /** Get the number of constraints which are associated with this body. */ 
    HP_Body_GetNumConstraints(bodyId : HP_BodyId): [Result, number];
    /** Get the handle to the constraint at index `index` on the body. */ 
    HP_Body_GetConstraint(bodyId : HP_BodyId, index : number): [Result, HP_ConstraintId];
    /** Get the bounding box of the body (in world space) */ 
    HP_Body_GetBoundingBox(bodyId : HP_BodyId): [Result, Aabb];
    /** Allocates a new handle for a constraint object, which limits the relative movement between two bodies. */ 
    HP_Constraint_Create(): [Result, HP_ConstraintId];
    /** Release the constraint handle, freeing it's memory if not in use. */ 
    HP_Constraint_Release(constraintId : HP_ConstraintId): Result;
    /** Set the `parent` body of the constraint. Limits are defined with respect to this body. */ 
    HP_Constraint_SetParentBody(constraint : HP_ConstraintId, body : HP_BodyId): Result;
    /** Get the parent body for a particular constraint. */ 
    HP_Constraint_GetParentBody(constraint : HP_ConstraintId): [Result, HP_BodyId];
    /** Set the "child" body of the constraint. This is the other body which the constraint is attached to. */ 
    HP_Constraint_SetChildBody(constraint : HP_ConstraintId, body : HP_BodyId): Result;
    /** Get the child body for a particular constraint. */ 
    HP_Constraint_GetChildBody(constraint : HP_ConstraintId): [Result, HP_BodyId];
    /** Configure the constraint space of the parent body (in parent body space).
     * @param pivot - origin of constraint space (in parent body space)
     * @param axisX - basis vector for constraint space X (in parent body space)
     * @param axisY - basis vector for constraint space Y (in parent body space)
     * A third basis vector is calculated perpendicular to axisX/axisY.
     * You must also configure the anchor in child space. */
    HP_Constraint_SetAnchorInParent(constraint : HP_ConstraintId, pivot : Vector3, axisX : Vector3, axisY : Vector3): Result;
    /** Configure the constraint space of the child body (in parent body space).
     * @param pivot - origin of constraint space (in child body space)
     * @param axisX - basis vector for constraint space X (in child body space)
     * @param axisY - basis vector for constraint space Y (in child body space)
     * A third basis vector is calculated perpendicular to axisX/axisY.
     * You must also configure the anchor in parent space. */
    HP_Constraint_SetAnchorInChild(constraint : HP_ConstraintId, pivot : Vector3, axisX : Vector3, axisY : Vector3): Result;
    /** Enables a constraint if isEnabled is non-zero. Requires both bodies to be in the same world to have an effect. */ 
    HP_Constraint_SetEnabled(constraint : HP_ConstraintId, isEnabled : number): Result;
    /** Whether the constraint is enabled or not. */ 
    HP_Constraint_GetEnabled(constraint : HP_ConstraintId): [Result, number];
    /** Sets an arbitrary user data on the constraint. Not used by the engine. */ 
    //HP_Constraint_SetUserData(constraint : HP_ConstraintId, userData : number): Result;
    /** Retrieve the user data associated with a constraint. */ 
    //HP_Constraint_GetUserData(constraint : HP_ConstraintId): [Result, number];
    /** By default, a constraint will not allow collisions to occur between the two connected bodies. A non-zero value for isEnabled will change that behaviour. */ 
    HP_Constraint_SetCollisionsEnabled(constraint : HP_ConstraintId, isEnabled : number): Result;
    /** Retrieve whether collisions are enabled for an individual constraint. */ 
    HP_Constraint_GetCollisionsEnabled(constraint : HP_ConstraintId): [Result, number];
    /** Adds a friction coefficient which resists movement along the specified axis. */ 
    HP_Constraint_SetAxisFriction(constraint : HP_ConstraintId, axis : ConstraintAxis, friction : number): Result;
    /** Get the friction coefficient associated with a particular axis for a constraint. */ 
    HP_Constraint_GetAxisFriction(constraint : HP_ConstraintId, axis : ConstraintAxis): [Result, number];
    /** Set the limit behaviour of the specified axis. See ConstraintAxisLimitMode for each effect. */ 
    HP_Constraint_SetAxisMode(constraint : HP_ConstraintId, axis : ConstraintAxis, limitMode : ConstraintAxisLimitMode): Result;
    /** Get the limit behaviour for the specified axis. */ 
    HP_Constraint_GetAxisMode(constraint : HP_ConstraintId, axis : ConstraintAxis): [Result, ConstraintAxisLimitMode];
    /** Set the minimum allowed signed movement along the specified axis. 
     * For linear axes, this value is in meters; for angular axes, this is in radians.
     * Only has an effect when ConstraintAxisLimitMode is LIMITED. */ 
    HP_Constraint_SetAxisMinLimit(constraint : HP_ConstraintId, axis : ConstraintAxis, minLimit : number): Result;
    /** Retrieve the minumum signed movement along the specified axis. */ 
    HP_Constraint_GetAxisMinLimit(constraint : HP_ConstraintId, axis : ConstraintAxis): [Result, number];
    /** Set the maximum allowed signed movement along the specified axis.
     * For linear axes, this value is in meters; for angular axes, this is in radians.
     * Only has an effect when ConstraintAxisLimitMode is LIMITED. */ 
    HP_Constraint_SetAxisMaxLimit(constraint : HP_ConstraintId, axis : ConstraintAxis, maxLimit : number): Result;
    /** Retrieve the maximum signed movement along the specified axis. */ 
    HP_Constraint_GetAxisMaxLimit(constraint : HP_ConstraintId, axis : ConstraintAxis): [Result, number];
    /** Enable or disable a motor on the specified axis. See ConstraintMotorType for information. */ 
    HP_Constraint_SetAxisMotorType(constraint : HP_ConstraintId, axis : ConstraintAxis, motorType : ConstraintMotorType): Result;
    /** Get the type of motor used on the specified axis. */ 
    HP_Constraint_GetAxisMotorType(constraint : HP_ConstraintId, axis : ConstraintAxis): [Result, ConstraintMotorType];
    /** Set the target for the constraint motor on the specified axis. The precise meaning of target depends on the type of the constraint motor. */ 
    HP_Constraint_SetAxisMotorTarget(constraint : HP_ConstraintId, axis : ConstraintAxis, target : number): Result;
    /** Get the target for the constraint motor on the specified axis. */ 
    HP_Constraint_GetAxisMotorTarget(constraint : HP_ConstraintId, axis : ConstraintAxis): [Result, number];
    /** Set the max force for the constraint motor on the specified axis.
     * For angular axes, `maxForce` is used as a max torque. */ 
    HP_Constraint_SetAxisMotorMaxForce(constraint : HP_ConstraintId, axis : ConstraintAxis, maxForce : number): Result;
    /** Get the max force (or torque) for a constraint motor on a particular axis. */ 
    HP_Constraint_GetAxisMotorMaxForce(constraint : HP_ConstraintId, axis : ConstraintAxis): [Result, number];
    /** Allocate a new handle for a world, which is the basis of a simulation. */ 
    HP_World_Create(): [Result, HP_WorldId];
    /** Releases a world handle, freeing any memory used. */ 
    HP_World_Release(world : HP_WorldId): Result;
    /** Returns the address of the world's body buffer, for use with
     * HP_Body_GetWorldTransformBuffer. This result can be invalidated if a
     * body is added to the world.
     */
    HP_World_GetBodyBuffer(world: HP_WorldId): [Result, number];
    /** Sets an arbitrary user data on the world. Not used by the engine. */ 
    //HP_World_SetUserData(world : HP_WorldId, userData : number): Result;
    /** Retrieve the user data associated with a world. */ 
    //HP_World_GetUserData(world : HP_WorldId): [Result, number];
    /** Set the global acceleration due to gravity of a world. This is applied to all DYNAMIC bodies each step. */ 
    HP_World_SetGravity(world : HP_WorldId, gravity : Vector3): Result;
    /** Adds a body to the world, where it will partake in the simulation in the next step. A body can only be in a single world at a time. */ 
    HP_World_AddBody(world : HP_WorldId, body : HP_BodyId, startAsleep: boolean): Result;
    /** Remove a body from the world. */ 
    HP_World_RemoveBody(world : HP_WorldId, body: HP_BodyId): Result;
    /** Return the number of bodies added to the world. */ 
    HP_World_GetNumBodies(world : HP_WorldId): [Result, number];
    /** Get the next (after prevBody) body that is added to the world. Pass a HP_BodyId with a value of 0 to get the first body in the list. */ 
    HP_World_GetNextBody(world : HP_WorldId, prevBody : HP_BodyId): [Result, HP_BodyId];
    /** Get the bounding box for all bodies contained in the world. */ 
    HP_World_GetBoundingBox(world : HP_WorldId): [Result, Aabb];
    /** Allocates a query collector with sufficient capacity to store the requested number of hits. */ 
    HP_QueryCollector_Create(hitCapacity : number): [Result, HP_CollectorId];
    /** Releases a query collector handle, returning the memory to the plugin. */ 
    HP_QueryCollector_Release(collector : HP_CollectorId): Result;
    /** Get the number of hits currently stored in the collector. */ 
    HP_QueryCollector_GetNumHits(collector : HP_CollectorId): [Result, number];
    /** Get the raycast result stored at hitIndex in the collector.
     * Only valid if the last query performed with this collector was a raycast. */ 
    HP_QueryCollector_GetCastRayResult(collector : HP_CollectorId, hitIndex : number): [Result, RayCastResult];
    /** Perform the raycast described by `query` against the bodies added to the world, storing the results in collector.
     * Collector will be cleared of any previous results. */ 
    HP_World_CastRayWithCollector(world : HP_WorldId, collector : HP_CollectorId, query : RayCastInput): Result;
    /** Simulate the world and advance time by `timestep` seconds. */ 
    HP_World_Step(world : HP_WorldId, timestep : number): Result;
    /** Get the first collision event generated by the previous world step. */ 
    HP_World_GetCollisionEvents(world : HP_WorldId): [Result, number];
    /** Get the next collision event, following previousEvent. */
    HP_World_GetNextCollisionEvent(world: number, previousEvent: number): number;
    /** Convert a collision event ID to a concrete type */
    HP_Event_AsCollision(eventId : number): [Result, CollisionEvent];

    /** Start recording performance counters for a single world */
    HP_Debug_StartRecordingStats(world: HP_WorldId): Result;
    /** Stop recording performance counters for a world.
     * This will call a method 'timerData' on the callback object, which can
     * retrieve statistics formatted in XML, for use with hkMonitor */
    HP_Debug_StopRecordingStats(world: HP_WorldId, callback: any): Result;
}

declare const HK : EmscriptenModuleFactory<HavokPhysicsWithBindings>;
export default HK