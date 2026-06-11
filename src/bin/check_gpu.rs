fn main() {
    println!("GPU Available: {}", wgsl_fft::GpuFft::is_gpu_available());
    match wgsl_fft::GpuFft::new() {
        Ok(_) => println!("GPU FFT Initialized Successfully!"),
        Err(e) => println!("GPU FFT Init Error: {}", e),
    }
}
