use passiveradar::dsp::fft::{FftEngine, DISABLE_GPU};
fn main() {
    DISABLE_GPU.store(false, std::sync::atomic::Ordering::SeqCst);
    let _engine = FftEngine::new(128);
    // backend is private, but we can't easily check it. Let's just run it.
}
