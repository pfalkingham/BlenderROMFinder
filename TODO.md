Worker-side keyframe payload improvement

Goal
----
Move as much of the pose -> channel conversion into the headless worker processes so the main process only merges channel values and writes batched F‑curves. This preserves correctness (set+read behavior) while maintaining scalability for millions of valid poses.

Motivation
----------
- Current accurate approach performs set+scene update+read-back on the main thread for each valid pose, which is correct but can be slow when the number of valid poses becomes very large.
- Headless workers already run Blender and compute collisions; they can also apply poses and record the actual channel values (location, rotation_euler/quaternion, custom props) and return these along with pose_index.
- The main process can then sort/merge by pose_index and perform a single batched write of F‑curve data (foreach_set), which is fast even for millions of points.

Detailed proposal
-----------------
1. Worker changes (headless script):
   - When a worker finds a valid pose, it should:
     - Apply the pose (pose_bone.matrix or obj.matrix_local)
     - Call a depsgraph/scene update (bpy.context.view_layer.update())
     - Read back the actual channel values for the target (use rotation mode to decide: `rotation_euler` or `rotation_quaternion` + `location` and any custom props like `input_rot_x`).
     - Include these values in the worker's `valids` payload as `channels`: {
         'location': [x, y, z],
         'rotation_euler': [x, y, z] OR 'rotation_quaternion': [w,x,y,z],
         'custom_props': {'input_rot_x': v, ...}
       }
     - Include `pose_index` (already present) so main thread can order results.
   - Keep payload compact (avoid repeating matrix if channels are provided).
   - Optionally make this configurable (worker returns channels only when `--emit-channels` flag or a prop toggle).

2. Main process changes:
   - `merge_worker_payload` should accept `channels` and append them to an in-memory buffer keyed by pose_index (e.g., `collected_channels` list or dict).
   - `start_keyframe_creation` and `process_keyframe_batch` should be adapted to read from `collected_channels` (already concretely decoded) and skip set+update entirely — simply batch the channel values into `collected_data` structures and write using the existing batched F‑curve writer `apply_collected_keyframes_5_0`.
   - Export CSV ordering: continue to sort by `pose_index` and write in serial order.

3. Backwards compatibility and toggles:
   - Add a prop flag (e.g. `headless_workers_emit_channels`) default False to enable/disable worker channel emission.
   - If disabled, retain existing behaviour (main thread sets pose and reads back).

Testing & Validation
--------------------
- Add an integration test with a small set of poses where a worker emits channels and the main thread writes F‑curves; compare the resulting F‑curves to a serial run using `keyframe_insert()` for the same poses.
- Verify CSV identicality and animation results in both methods.

Risks & Notes
-------------
- Worker must have access to the same scene state (objects, parents, bone names, rotation mode settings) — ensure the headless blend file includes required objects and settings or pass props via JSON as already implemented.
- Quaternion sign/continuity: ensure workers write quaternion channels consistently (wxyz) and main thread writes using the same order.
- Payload size: channel payloads increase message size; consider compression or chunking for extreme workloads.

Implementation estimate
-----------------------
- Worker-side changes: small-to-moderate (1–2 days to implement and validate)
- Main-thread merge/write changes: small (half-day)
- Tests and docs: small (half-day)

Status
------
Proposed — needs prioritization. If you want, I can implement this next (requires adding a toggle and worker-side channel emission and updating merging/writing to use emitted channels).