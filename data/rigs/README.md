# Sample rigs

## `humanoid.rig.json`

A synthetic humanoid: 1.75 m tall, 75 kg, standing with its arms hanging down. It faces +Y with +X to its right and Z up, matching OpenMW and Bethesda content. No game skeleton and no game-derived rest pose is used. Bone names are generic.

- **Bones:** 21. They are Pelvis, Spine, Chest, Neck and Head, plus on each side UpperArm, Forearm, Hand, Fingers, Thigh, Shin and Foot. Every rest rotation is the identity, so each bone's axes match the world axes in the rest pose.
- **Bodies:** 16: three trunk bodies (pelvis, spine, chest, after de Leva's three trunk segments), the head, and six per side. Neck and Fingers are bones without bodies, so the head's joint connects it to the chest.
- **Profile:** `data/profiles/humanoid-default.profile.json`. Its gains are placeholders until M3.

### Masses (sourced)

Each segment mass is the male mass fraction from de Leva, P. (1996), "Adjustments to Zatsiorsky-Seluyanov's segment inertia parameters", *Journal of Biomechanics* 29(9), 1223–1230 (Table 4), times 75 kg. The fractions sum to 100 %, and the sample totals 75.0 kg.

| Body | de Leva segment | Fraction | Mass (kg) |
| --- | --- | --- | --- |
| Head | head and neck | 6.94 % | 5.205 |
| Chest | upper trunk | 15.96 % | 11.97 |
| Spine | middle trunk | 16.33 % | 12.2475 |
| Pelvis | lower trunk | 11.17 % | 8.3775 |
| UpperArm (each) | upper arm | 2.71 % | 2.0325 |
| Forearm (each) | forearm | 1.62 % | 1.215 |
| Hand (each) | hand | 0.61 % | 0.4575 |
| Thigh (each) | thigh | 14.16 % | 10.62 |
| Shin (each) | shank | 4.33 % | 3.2475 |
| Foot (each) | foot | 1.37 % | 1.0275 |

Mass is placed at each body's shape centre. de Leva's centre-of-mass positions and inertia are not used yet: the physics interface takes a mass, and Jolt derives inertia from the shape.

### Lengths and shapes (estimates, not sourced)

Bone offsets and shape sizes are round-number estimates. They are chosen so the figure stands 1.75 m tall, with the head top at about 1.755 m and the soles at z = 0. They are not taken from a published table. They only need to be plausible for the sandbox; real characters supply their own skeletons.

### Joint limits (sourced, with stated approximations)

The ranges are the normal ranges of motion published by the American Academy of Orthopaedic Surgeons: Greene, W. B. & Heckman, J. D. (eds.) (1994), *The Clinical Measurement of Joint Motion*, AAOS. The values below are as widely cited from that source. **They were not checked against the book itself, so treat them as unverified until someone does.**

Version 1 joints have symmetric swing cones centred on the rest pose (`docs/formats/rig.md`). A range that is wider one way than the other is therefore approximated, and those rows are marked *approx.*

| Joint (child bone) | Published range | In the file |
| --- | --- | --- |
| Spine, Chest (each) | trunk flexion 80°, extension 25°, side bend 35°, rotation 45° | each joint takes half: normal cone 0.46 (≈26°, *approx.*), plane cone 0.31 (≈17.5°), twist ±0.39 (≈22.5°) |
| Head (from chest) | neck flexion 45°, extension 45°, side bend 45°, rotation 60° | normal 0.785 (45°), plane 0.785 (45°), twist ±1.047 (60°) |
| UpperArm (shoulder) | flexion 180°, extension 60°, abduction 180°, internal rotation 70°, external rotation 90° | normal 1.571 (90°, *approx.*), plane 1.571 (90°, *approx.*), twist ±1.222 (70°, the smaller rotation, so the left and right sides need no sign convention yet) |
| Forearm (elbow, hinge about +X) | flexion 0–150° | twist 0 to 2.618 (150°), cones 0 |
| Hand (wrist) | flexion 80°, extension 70°, radial deviation 20°, ulnar deviation 30°; forearm pronation and supination 80° each | normal 1.309 (75°, *approx.*), plane 0.436 (25°, *approx.*), twist ±1.396 (80°; forearm rotation is placed at the wrist because the elbow is a hinge) |
| Thigh (hip) | flexion 120°, extension 30°, abduction 45°, adduction 30°, internal and external rotation 45° each | normal 1.309 (75°, *approx.*), plane 0.654 (37.5°, *approx.*), twist ±0.785 (45°) |
| Shin (knee, hinge about +X) | flexion 0–135° | twist −2.356 to 0 (135°; with the shin hanging down, rotating about +X by a negative angle swings the foot backwards), cones 0 |
| Foot (ankle) | dorsiflexion 20°, plantar flexion 50°, inversion 35°, eversion 15° | normal 0.611 (35°, *approx.*), plane 0.436 (25°, *approx.*), twist ±0.175 (10°, an estimate; the published table has no matching motion) |

The hinge signs follow the right-hand rule about +X in the rest pose. Whether Jolt's swing-twist constraint measures twist with the same sign is checked in M3, when the rig first drives a ragdoll.
