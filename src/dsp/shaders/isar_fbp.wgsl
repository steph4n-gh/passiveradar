struct IsarParams {
    grid_size: u32,
    num_angles: u32,
    num_bins: u32,
    pad: u32,
};

@group(0) @binding(0) var<uniform> params: IsarParams;
@group(0) @binding(1) var<storage, read> profiles: array<f32>;      // [angle * num_bins + bin]
@group(0) @binding(2) var<storage, read> angles: array<f32>;        // cos/sin pairs: [cos0, sin0, cos1, sin1, ...]
@group(0) @binding(3) var<storage, read_write> image: array<f32>;   // [x * grid_size + y]

@compute @workgroup_size(16, 16)
fn backproject(@builtin(global_invocation_id) id: vec3<u32>) {
    let x_idx = id.x;
    let y_idx = id.y;
    if (x_idx >= params.grid_size || y_idx >= params.grid_size) { return; }

    let half_grid = f32(params.grid_size) / 2.0;
    let half_bins = f32(params.num_bins) / 2.0;
    let x = (f32(x_idx) - half_grid) / half_grid;
    let y = (f32(y_idx) - half_grid) / half_grid;

    var pixel_val = 0.0;
    for (var a = 0u; a < params.num_angles; a = a + 1u) {
        let cos_t = angles[a * 2u];
        let sin_t = angles[a * 2u + 1u];
        let rho = x * cos_t + y * sin_t;
        let bin = rho * half_bins + half_bins;

        if (bin >= 0.0 && bin < f32(params.num_bins - 1u)) {
            let bin_floor = u32(floor(bin));
            let bin_ceil = min(bin_floor + 1u, params.num_bins - 1u);
            let frac = bin - f32(bin_floor);
            let val = (1.0 - frac) * profiles[a * params.num_bins + bin_floor]
                    + frac * profiles[a * params.num_bins + bin_ceil];
            pixel_val = pixel_val + val;
        }
    }

    image[x_idx * params.grid_size + y_idx] = pixel_val / f32(params.num_angles);
}
