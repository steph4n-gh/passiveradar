use num_complex::Complex;
use passiveradar::dsp::caf::CafEngine;
use passiveradar::dsp::fft::{FftEngine, DISABLE_GPU};
use std::sync::atomic::Ordering;
use rand::Rng;
use std::time::Instant;

#[test]
fn test_fft_cpu_gpu_equivalence_and_performance() {
    let fft_size = 4096;
    
    // CPU FFT
    DISABLE_GPU.store(true, Ordering::SeqCst);
    let mut cpu_fft = FftEngine::new(fft_size);
    
    // GPU FFT
    DISABLE_GPU.store(false, Ordering::SeqCst);
    let mut gpu_fft = FftEngine::new(fft_size);
    
    let mut rng = rand::thread_rng();
    
    // Generate some random signal
    let mut signal = vec![Complex::new(0.0, 0.0); fft_size];
    for i in 0..fft_size {
        signal[i] = Complex::new(rng.gen::<f32>() - 0.5, rng.gen::<f32>() - 0.5);
    }
    
    // Test Correctness
    cpu_fft.feed(&signal);
    let cpu_mag = cpu_fft.next_frame(fft_size).unwrap();
    
    gpu_fft.feed(&signal);
    let gpu_mag = gpu_fft.next_frame(fft_size).unwrap();
    
    for i in 0..fft_size {
        let diff = (cpu_mag[i] - gpu_mag[i]).abs();
        assert!(diff < 1e-3, "FFT Correctness mismatch at index {}: CPU = {}, GPU = {}, Diff = {}", i, cpu_mag[i], gpu_mag[i], diff);
    }

    // Performance Test
    let num_iters = 100;
    
    // Benchmark CPU
    let start_cpu = Instant::now();
    for _ in 0..num_iters {
        cpu_fft.feed(&signal);
        let _ = cpu_fft.next_frame(fft_size).unwrap();
    }
    let cpu_duration = start_cpu.elapsed();

    // Benchmark GPU
    let start_gpu = Instant::now();
    for _ in 0..num_iters {
        gpu_fft.feed(&signal);
        let _ = gpu_fft.next_frame(fft_size).unwrap();
    }
    let gpu_duration = start_gpu.elapsed();

    println!("FFT CPU Time: {:?}", cpu_duration);
    println!("FFT GPU Time: {:?}", gpu_duration);
}

#[test]
fn test_caf_cpu_gpu_equivalence_and_performance() {
    let fft_size = 4096;
    let max_delay = 10;
    
    // CPU CAF
    DISABLE_GPU.store(true, Ordering::SeqCst);
    let mut cpu_caf = CafEngine::new(fft_size);
    
    // GPU CAF
    DISABLE_GPU.store(false, Ordering::SeqCst);
    let mut gpu_caf = CafEngine::new(fft_size);
    
    let mut rng = rand::thread_rng();
    
    let n_samples = 8192;
    let mut reference = vec![Complex::new(0.0, 0.0); n_samples];
    let mut surveillance = vec![Complex::new(0.0, 0.0); n_samples];
    for i in 0..n_samples {
        reference[i] = Complex::new(rng.gen::<f32>() - 0.5, rng.gen::<f32>() - 0.5);
        surveillance[i] = Complex::new(rng.gen::<f32>() - 0.5, rng.gen::<f32>() - 0.5);
    }
    
    // Correctness
    let cpu_caf_result = cpu_caf.compute(&reference, &surveillance, max_delay);
    let gpu_caf_result = gpu_caf.compute(&reference, &surveillance, max_delay);
    
    for d in 0..max_delay {
        for f in 0..fft_size {
            let diff = (cpu_caf_result[d][f] - gpu_caf_result[d][f]).abs();
            // Scale diff based on values, or use a larger threshold since CAF might amplify differences
            assert!(diff < 1e-1, "CAF Correctness mismatch at delay {}, bin {}: CPU = {}, GPU = {}, Diff = {}", d, f, cpu_caf_result[d][f], gpu_caf_result[d][f], diff);
        }
    }

    // Performance Test
    let num_iters = 10;
    
    let start_cpu = Instant::now();
    for _ in 0..num_iters {
        let _ = cpu_caf.compute(&reference, &surveillance, max_delay);
    }
    let cpu_duration = start_cpu.elapsed();

    let start_gpu = Instant::now();
    for _ in 0..num_iters {
        let _ = gpu_caf.compute(&reference, &surveillance, max_delay);
    }
    let gpu_duration = start_gpu.elapsed();

    println!("CAF CPU Time: {:?}", cpu_duration);
    println!("CAF GPU Time: {:?}", gpu_duration);
}
