struct Params {
    phase: f32,
    phase_step: f32,
    decimation_factor: u32,
    num_taps: u32,
    input_len: u32,
    counter: u32,
};

@group(0) @binding(0) var<uniform> params: Params;
@group(0) @binding(1) var<storage, read> taps: array<f32>;
@group(0) @binding(2) var<storage, read> input_samples: array<vec2<f32>>;
@group(0) @binding(3) var<storage, read> history_in: array<vec2<f32>>;
@group(0) @binding(4) var<storage, read_write> output_samples: array<vec2<f32>>;
@group(0) @binding(5) var<storage, read_write> history_out: array<vec2<f32>>;

fn get_mixed_sample(idx: u32) -> vec2<f32> {
    let history_len = params.num_taps - 1u;
    if (idx < history_len) {
        return history_in[idx];
    } else {
        let k = idx - history_len;
        let input_sample = input_samples[k];
        let theta = params.phase + f32(k) * params.phase_step;
        let cos_theta = cos(theta);
        let sin_theta = sin(theta);
        return vec2<f32>(
            input_sample.x * cos_theta + input_sample.y * sin_theta,
            input_sample.y * cos_theta - input_sample.x * sin_theta
        );
    }
}

@compute @workgroup_size(256)
fn mix_and_decimate(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let out_idx = global_id.x;
    
    let history_len = params.num_taps - 1u;
    let total_len = history_len + params.input_len;
    let start_idx = params.counter + out_idx * params.decimation_factor;
    
    if (start_idx + params.num_taps > total_len) {
        return;
    }
    
    var sum = vec2<f32>(0.0, 0.0);
    for (var t = 0u; t < params.num_taps; t = t + 1u) {
        let sample_val = get_mixed_sample(start_idx + t);
        let tap_val = taps[params.num_taps - 1u - t];
        sum = sum + sample_val * tap_val;
    }
    
    output_samples[out_idx] = sum;
}

@compute @workgroup_size(256)
fn copy_history(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let idx = global_id.x;
    let history_len = params.num_taps - 1u;
    if (idx >= history_len) {
        return;
    }
    
    let src_idx = params.input_len + idx;
    history_out[idx] = get_mixed_sample(src_idx);
}
