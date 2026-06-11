use num_complex::Complex;
use passiveradar::dsp::caf::CafEngine;
use passiveradar::dsp::fft::{FftEngine, DISABLE_GPU};
use rand::Rng;
use std::sync::atomic::Ordering;
use std::time::Instant;

fn generate_random_signal(len: usize) -> Vec<Complex<f32>> {
    let mut rng = rand::thread_rng();
    let mut sig = Vec::with_capacity(len);
    for _ in 0..len {
        sig.push(Complex::new(rng.gen::<f32>() - 0.5, rng.gen::<f32>() - 0.5));
    }
    sig
}

fn verify_fft_correctness_and_perf() {
    let fft_size = 4096;
    let signal = generate_random_signal(fft_size * 4); // Enough for a few frames

    // ---- GPU FFT ----
    DISABLE_GPU.store(false, Ordering::SeqCst);
    let mut engine_gpu = FftEngine::new(fft_size);
    engine_gpu.feed(&signal);
    
    let t0 = Instant::now();
    let gpu_frame1 = engine_gpu.next_frame(fft_size / 2).unwrap();
    let gpu_frame2 = engine_gpu.next_frame(fft_size / 2).unwrap();
    let gpu_dur = t0.elapsed();

    // ---- CPU FFT ----
    DISABLE_GPU.store(true, Ordering::SeqCst);
    let mut engine_cpu = FftEngine::new(fft_size);
    engine_cpu.feed(&signal);
    
    let t0 = Instant::now();
    let cpu_frame1 = engine_cpu.next_frame(fft_size / 2).unwrap();
    let cpu_frame2 = engine_cpu.next_frame(fft_size / 2).unwrap();
    let cpu_dur = t0.elapsed();

    // ---- Correctness Check ----
    let mut max_err = 0.0f32;
    for i in 0..fft_size {
        let err1 = (gpu_frame1[i] - cpu_frame1[i]).abs();
        let err2 = (gpu_frame2[i] - cpu_frame2[i]).abs();
        if err1 > max_err { max_err = err1; }
        if err2 > max_err { max_err = err2; }
    }
    
    println!("FFT Max Error: {}", max_err);
    println!("FFT GPU Time: {:?}", gpu_dur);
    println!("FFT CPU Time: {:?}", cpu_dur);

    // Assert that the difference is within float precision (e.g. 1e-4)
    assert!(max_err < 1e-3, "GPU FFT output deviates from CPU FFT by {}", max_err);
    // Performance sanity check (GPU shouldn't be insanely slower than CPU for 4096 size, or at least we print it out)
}

fn verify_caf_correctness_and_perf() {
    let fft_size = 4096;
    let max_delay = 32;
    let len = fft_size + max_delay;
    let ref_sig = generate_random_signal(len);
    let surv_sig = generate_random_signal(len);

    // ---- GPU CAF ----
    DISABLE_GPU.store(false, Ordering::SeqCst);
    let mut caf_gpu = CafEngine::new(fft_size);
    
    let t0 = Instant::now();
    let gpu_caf = caf_gpu.compute(&ref_sig, &surv_sig, max_delay);
    let gpu_dur = t0.elapsed();

    // ---- CPU CAF ----
    DISABLE_GPU.store(true, Ordering::SeqCst);
    let mut caf_cpu = CafEngine::new(fft_size);
    
    let t0 = Instant::now();
    let cpu_caf = caf_cpu.compute(&ref_sig, &surv_sig, max_delay);
    let cpu_dur = t0.elapsed();

    // ---- Correctness Check ----
    let mut max_err = 0.0f32;
    for d in 0..max_delay {
        for b in 0..fft_size {
            let err = (gpu_caf[d][b] - cpu_caf[d][b]).abs();
            if err > max_err { max_err = err; }
        }
    }

    println!("CAF Max Error: {}", max_err);
    println!("CAF GPU Time: {:?}", gpu_dur);
    println!("CAF CPU Time: {:?}", cpu_dur);

    assert!(max_err < 1e-1, "GPU CAF output deviates from CPU CAF by {}", max_err);
}

fn main() {
    println!("Running FFT Verification...");
    verify_fft_correctness_and_perf();
    println!("Running CAF Verification...");
    verify_caf_correctness_and_perf();
    println!("All done.");
}
