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
