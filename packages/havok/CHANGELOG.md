# Changelog

## 1.0.1

- Fixing the file references in package.json
- Added missing License in package

## 1.1.0

Adds a variety of improvements to the WASM. Interface is backwards-compatible.

- Fixes crash when adding a constraint between a body and itself
- Improves convex hull generation when the shape is very small (~1cm)
- Flush broadphase addition queue before performing raycasts (Fixes When doing an intersection query, update all the bodies that were added on that phase #1)
- Add support for triggers (Fixes Add intersection triggers to the WASM #2)
- Add methods to control activation state of bodies
- Add methods to control variable timestep utility (allows for improved behaviour at step frequencies faster than 60Hz)
- Add support for soft constraints

## 1.1.1

- Enable motion welding for dynamic-static pairs - improves quality of object sliding over meshes
- Fix bug where contact-finished events were not exported

## 1.1.2

- Fix a typescript declaration issue

## 1.1.3

- Allows unlimited unique combinations of linear/angular damping and gravity factor

## 1.1.4

- Tweak motion welding settings to remove artifacts when spheres roll along meshes

## 1.2.0

- Add an interface to apply an angular impulse to bodies
- Add a constraint motor to model a spring damper with individual position/velocity targets
- Optimize updating constraints when changing many settings at once
- Fix bug where changing constraint damping/stiffness had no effect until other constraint settings were changed
- Fix bug where a small amount of angular damping was always applied to bodies, even if a lower value was requested
- Fix bug where deeply-penetrating triggers could raise contact events

## 1.2.1

- Fix crash when raising trigger/contact events for bodies which have been destroyed

## 1.3.0

- Add interface to perform "closest point to position" query
- Add interface to perform "closest point to shape" query
- Add interface to perform a "shape cast" query
- Add additional filtering options to "ray cast" query
- Rename spring motors for clarity
- Fix bug where a contact event could be raised between a kinematic-static or kinematic-kinematic pair
- Fix bug where changing constraint parameters would not activate sleeping bodies
- Fix bug where triangle indices weren't reported for meshes inside a container shape
- Re-enable a small amount of angular damping per-body by default
