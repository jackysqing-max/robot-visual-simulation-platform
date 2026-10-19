# Student implementation and application integration plan

The tutorial establishes the motion-control chain so students can independently send joint goals and end-effector paths before progressing to the medical instrument sorting application. Later deliverables below are planned work. Mark an item complete only when code and reproducible execution records support it.

Repository: https://github.com/jackysqing-max/robot-visual-simulation-platform

## Tutorial steps T0 to T5

| Step | Work | Evidence to retain |
| --- | --- | --- |
| T0 Environment | Build both ROS packages; start the default PyBullet simulator and controllers | Setup output, package list, active controller list |
| T1 Joint control | Run the joint demo and one manual JointState goal | Input, action result, measured joint error, JSON report |
| T2 End-effector kinematics | Explain the base, link7, quaternions, and IK; run the Cartesian demo | Target and measured paths, position and orientation errors |
| T3 ROS interfaces | Publish goals, correlate status IDs, demonstrate busy rejection and cancellation | Topic graph, status log, rejection and cancellation records |
| T4 Existing simulation | Connect the existing PyBullet implementation using `sim_backend:=existing` | Separate joint and Cartesian results for both backends |
| T5 Application adapter | Send validated grasp and placement poses to the motion interface | Field definitions, frame/TCP conversion, reproducible task sequence |

Review T0 through T4 in order. Begin T5 after M2/M3 establish coordinate and grasp outputs. Each step should add one observable capability. Fix a failed step before connecting more components.

## Application milestones M1 to M5

### M1 Instruction to segmentation

Connect `/llm/request_json`, `/llm/response_json`, and SAM3. Parse the model's returned text and validate the action, target, and destination. HTTP success alone does not establish a valid task. Match LLM requests and results by `request_id`.

SAM3 has no task ID in the integration described here. Process requests serially, confirm the active prompt, and accept only masks associated with newly captured RGB images. Correlate source images through their headers; message order alone cannot distinguish an old mask from a new result. Keep the robot stationary during this stage.

Deliverables: an instruction-to-mask application node, task JSON schema, success/rejection/error logs, and per-case results. The tutorial motion node does not need raw natural-language instructions.

### M2 RGB-D and base coordinates

Match aligned depth and CameraInfo to the mask's source RGB acquisition time. Filter invalid depth and recover camera-frame 3D points. Calibrate the camera-to-base transform, use TF at the corresponding capture time, document the base-to-tool TCP definition, and validate with independent measurement points.

Deliverables: paired frames, point clouds, valid-depth rate, time offsets, TF tree, calibration values, and independent error measurements. For the default tutorial model, convert a desired tool TCP pose to the model's link7 pose before sending it. Configured workbench projects instead accept the configured TCP directly; follow [workbench frame conventions](workbench.md).

### M3 Grasp generation and feasibility

Generate candidates from graspable handle regions. Retain request/capture provenance, frame, position, quaternion, gripper width, and approach direction. Check workspace, IK, joint limits, collisions, and gripper opening for the candidate and the full approach/transfer path before selecting a goal.

The tutorial's numerical IK is a learning baseline. A complete grasp also needs collision models, TCP, gripper geometry, and object geometry. An IK solution alone does not validate a grasp.

Deliverables: candidates with rejection reasons, complete constraints, approach-pose visualization, and per-scene results. Start T5 after this review passes.

### M4 Execution and result verification

Implement a state machine: acquire a fresh scene, validate the task, move to pregrasp, approach, close the gripper, lift, transfer, release, and verify. Arm segments call the tutorial pose/path interface or standard trajectory action. A separate interface controls the gripper.

Record the mapping between node-assigned motion request IDs and application task IDs. Advance only after a matching final success event. Keep explicit failure, busy, and cancellation states; do not skip to release or task completion. Verify the physical result after lifting and after placement.

Deliverables: state machine, gripper and instrument models, contact-simulation or real-robot evidence, and grasp/placement verification. The current arm-motion demos do not complete this milestone.

### M5 Fixed evaluation and reproduction

Freeze the configuration and evaluate scenes and instructions that were not used for tuning. Record first attempts and subsequent recovery separately. Measure target selection, grasp success, complete-task success, control rejections, 3D error, and latency. Preserve each failure instead of overwriting it with a later successful retry.

Deliverables: fixed task list, versioned configuration, raw per-case results, summary statistics, representative failures, and continuous video. Trial counts and pass rates in the accompanying student guide are proposed teaching targets. Use them as local acceptance criteria only after instructor agreement; they are not current measured results.

## Application adapter

Keep the responsibilities of `llm_ros`, `sam3_ros`, and the motion tutorial separate. Add an application package for task correlation and the state machine. A proposed input is:

```json
{
  "task_id": "trial_001",
  "action": "pick_and_place",
  "target_prompt": "surgical scissors",
  "destination": "tray_1"
}
```

This is an application schema to implement, not an existing LLM or motion-topic message definition. Send validated prompts to `/sam3/prompt`. Submit arm goals to `/lbr_tutorial/pose_goal` or `/lbr_tutorial/cartesian_path` only after M2/M3 provide reliable poses in the required base frame.

| Application field | Record |
| --- | --- |
| `task_id` | Instruction and complete task ID |
| `capture_id` | Shared provenance of RGB, mask, and depth |
| `candidate_id` | Selected grasp candidate |
| `motion_request_id` | Request ID returned by the tutorial motion node |
| `stage` | pregrasp / approach / lift / transfer / release |
| `outcome` | Physical verification result and failure reason |

## Review records

For each task, record its ID, objective, owner, reviewer, agreed review date, inputs, implementation steps, acceptance criteria, code/evidence links, and status. Use Planned, In progress, Ready for review, or Completed. Mark Completed after reproducible evidence has been reviewed.

The first motion review should cover T0/T1; the second T2/T3; the third should compare T4 backends and confirm the application interface. Continue application work through M1, M2, M3, M4, and M5 in order. Set dates according to camera, gripper, and experiment availability.
