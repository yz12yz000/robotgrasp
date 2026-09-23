These are two independent, stationary RGB/depth observations captured on
2026-09-23 using `tools/capture_grasp_observation.py`. Only the segmented bowl
XYZ points in `base_link` are included (metres, object 0 green, object 1 white).
The full RGB, CameraInfo, acquisition timestamps and TF are recorded locally in
`debug_output/rim_geometry_20260923/frame01` and `frame02`.

The old single-anchor wall fit rejects both bowls. These fixtures verify that
the supported-contact search finds measured wall support without relaxing the
40 degree limit. They have no independently measured contact/normal ground
truth and do not demonstrate physical grasp success or calibration accuracy.

`white_green_depth_tail_1.npz` and `white_green_depth_tail_2.npz` record the
repositioned green and white bowls in `live_acceptance` and `white_green_preview`
under `debug_output/planning_diagnosis_20260923`. The white bowl includes a
connected high depth streak. These fixtures test repeatable contacts and exclude
the streak from candidate anchors; they do not provide independent metric truth.
