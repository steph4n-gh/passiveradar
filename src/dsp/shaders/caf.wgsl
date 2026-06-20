struct CafParams {
    num_samples: u32,
    max_delay: u32,
    num_doppler_bins: u32,
    doppler_step: f32,
    block_size: u32,
    pad0: u32,
};

@group(0) @binding(0) var<uniform> params: CafParams;
@group(0) @binding(1) var<storage, read> surv_samples: array<vec2<f32>>;
@group(0) @binding(2) var<storage, read> ref_samples: array<vec2<f32>>;
@group(0) @binding(3) var<storage, read_write> output_surface: array<f32>;

const PI: f32 = 3.14159265358979323846;

fn complex_mul(a: vec2<f32>, b: vec2<f32>) -> vec2<f32> {
    return vec2<f32>(
        a.x * b.x - a.y * b.y,
        a.x * b.y + a.y * b.x,
    );
}

@compute @workgroup_size(256)
fn caf_compute(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let cell_idx = global_id.x;
    let total_cells = params.max_delay * params.num_doppler_bins;
    if (cell_idx >= total_cells) {
        return;
    }

    let delay_idx = cell_idx / params.num_doppler_bins;
    let doppler_idx = cell_idx % params.num_doppler_bins;
    let delay = delay_idx;

    let fd = f32(i32(doppler_idx) - i32(params.num_doppler_bins / 2u)) * params.doppler_step;

    var sum_re = 0.0;
    var sum_im = 0.0;

    for (var n = 0u; n < params.block_size; n = n + 1u) {
        if (n + delay < params.num_samples) {
            let phase = -2.0 * PI * fd * f32(n) / f32(params.num_samples);
            let doppler_phasor = vec2<f32>(cos(phase), sin(phase));

            let s = surv_samples[n];
            let r = ref_samples[n + delay];
            let r_conj = vec2<f32>(r.x, -r.y);

            let product = complex_mul(s, r_conj);
            let rotated = complex_mul(product, doppler_phasor);

            sum_re = sum_re + rotated.x;
            sum_im = sum_im + rotated.y;
        }
    }

    output_surface[cell_idx] = sum_re * sum_re + sum_im * sum_im;
}
