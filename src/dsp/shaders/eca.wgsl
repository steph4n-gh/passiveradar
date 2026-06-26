struct EcaParams {
    num_taps: u32,
    max_iterations: u32,
    n_samples: u32,
    pad: u32,
};

@group(0) @binding(0) var<uniform> params: EcaParams;
@group(0) @binding(1) var<storage, read> input_buf: array<vec2<f32>>;
@group(0) @binding(2) var<storage, read> history_buf: array<vec2<f32>>;
@group(0) @binding(3) var<storage, read_write> output_buf: array<vec2<f32>>;
@group(0) @binding(4) var<storage, read_write> r_buf: array<vec2<f32>>;
@group(0) @binding(5) var<storage, read_write> q_buf: array<vec2<f32>>;
@group(0) @binding(6) var<storage, read_write> w_buf: array<vec2<f32>>;
@group(0) @binding(7) var<storage, read_write> p_buf: array<vec2<f32>>;
@group(0) @binding(8) var<storage, read_write> s_new_buf: array<vec2<f32>>;

var<workgroup> scratch: array<f32, 256>;

fn complex_mul(a: vec2<f32>, b: vec2<f32>) -> vec2<f32> {
    return vec2<f32>(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x);
}

fn complex_conj(a: vec2<f32>) -> vec2<f32> {
    return vec2<f32>(a.x, -a.y);
}

fn complex_norm_sqr(a: vec2<f32>) -> f32 {
    return a.x * a.x + a.y * a.y;
}

fn get_delayed_input(i: u32, delay: u32) -> vec2<f32> {
    if (i >= delay) {
        return input_buf[i - delay];
    } else {
        return history_buf[params.num_taps - delay + i];
    }
}

fn reduce_normq_sq(local_id: u32) -> f32 {
    var sum = 0.0;
    for (var i = local_id; i < params.n_samples; i += 256u) {
        sum += complex_norm_sqr(q_buf[i]);
    }
    scratch[local_id] = sum;
    workgroupBarrier();

    for (var stride = 128u; stride > 0u; stride /= 2u) {
        if (local_id < stride) {
            scratch[local_id] += scratch[local_id + stride];
        }
        workgroupBarrier();
    }
    return scratch[0];
}

@compute @workgroup_size(256)
fn eca_cancel(@builtin(local_invocation_id) local_id_vec: vec3<u32>) {
    let local_id = local_id_vec.x;

    // 1. Initialize buffers on parallel threads
    for (var i = local_id; i < params.n_samples; i += 256u) {
        r_buf[i] = input_buf[i];
    }
    for (var k = local_id; k < params.num_taps; k += 256u) {
        w_buf[k] = vec2<f32>(0.0, 0.0);
    }
    workgroupBarrier();

    // 2. Compute initial p = X^H * r
    for (var k = local_id; k < params.num_taps; k += 256u) {
        var sum = vec2<f32>(0.0, 0.0);
        for (var i = 0u; i < params.n_samples; i = i + 1u) {
            let val = get_delayed_input(i, k + 1u);
            sum = sum + complex_mul(complex_conj(val), r_buf[i]);
        }
        p_buf[k] = sum;
    }
    workgroupBarrier();

    // 3. Compute initial norms_sq of p
    var norms_sq = 0.0;
    if (local_id == 0u) {
        for (var k = 0u; k < params.num_taps; k = k + 1u) {
            norms_sq = norms_sq + complex_norm_sqr(p_buf[k]);
        }
        scratch[0] = norms_sq;
    }
    workgroupBarrier();
    norms_sq = scratch[0];
    workgroupBarrier();

    // 4. CG Iteration Loop
    for (var iter = 0u; iter < params.max_iterations; iter = iter + 1u) {
        if (norms_sq < 1e-10) { break; }

        // Compute q = X * p
        for (var i = local_id; i < params.n_samples; i += 256u) {
            var sum = vec2<f32>(0.0, 0.0);
            for (var k = 0u; k < params.num_taps; k = k + 1u) {
                let val = get_delayed_input(i, k + 1u);
                sum = sum + complex_mul(val, p_buf[k]);
            }
            q_buf[i] = sum;
        }
        workgroupBarrier();

        // Compute normq_sq of q
        let normq_sq = reduce_normq_sq(local_id);
        workgroupBarrier();

        if (normq_sq < 1e-15) { break; }

        let alpha = norms_sq / normq_sq;

        // Update weights and residuals
        for (var k = local_id; k < params.num_taps; k += 256u) {
            w_buf[k] = w_buf[k] + p_buf[k] * alpha;
        }
        for (var i = local_id; i < params.n_samples; i += 256u) {
            r_buf[i] = r_buf[i] - q_buf[i] * alpha;
        }
        workgroupBarrier();

        // Compute s_new = X^H * r
        for (var k = local_id; k < params.num_taps; k += 256u) {
            var sum = vec2<f32>(0.0, 0.0);
            for (var i = 0u; i < params.n_samples; i = i + 1u) {
                let val = get_delayed_input(i, k + 1u);
                sum = sum + complex_mul(complex_conj(val), r_buf[i]);
            }
            s_new_buf[k] = sum;
        }
        workgroupBarrier();

        // Compute norms_new_sq of s_new
        var norms_new_sq = 0.0;
        if (local_id == 0u) {
            for (var k = 0u; k < params.num_taps; k = k + 1u) {
                norms_new_sq = norms_new_sq + complex_norm_sqr(s_new_buf[k]);
            }
            scratch[0] = norms_new_sq;
        }
        workgroupBarrier();
        norms_new_sq = scratch[0];
        workgroupBarrier();

        let beta = norms_new_sq / norms_sq;

        // Update p
        for (var k = local_id; k < params.num_taps; k += 256u) {
            p_buf[k] = s_new_buf[k] + p_buf[k] * beta;
        }
        workgroupBarrier();

        norms_sq = norms_new_sq;
    }

    // 5. Output result
    for (var i = local_id; i < params.n_samples; i += 256u) {
        output_buf[i] = r_buf[i];
    }
}
