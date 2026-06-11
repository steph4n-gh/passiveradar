use num_complex::Complex;
use rand::{Rng, SeedableRng};
use rand_distr::{Distribution, Normal};
use std::error::Error;
use std::time::Instant;

pub trait SdrSource: Send {
    fn start(&mut self) -> Result<(), Box<dyn Error>>;
    fn read(&mut self, buffer: &mut [Complex<f32>]) -> Result<usize, Box<dyn Error>>;
    fn stop(&mut self) -> Result<(), Box<dyn Error>>;
    fn set_frequency(&mut self, freq: f64) -> Result<(), Box<dyn Error>>;
    fn set_sample_rate(&mut self, rate: f64) -> Result<(), Box<dyn Error>>;
    fn get_sample_rate(&self) -> f64;
    fn get_frequency(&self) -> f64;
    fn set_speed_factor(&mut self, _speed: f64) -> Result<(), Box<dyn Error>> { Ok(()) }
}

// =========================================================================
// 1. Physical SoapySDR Source
// =========================================================================
pub struct SoapySdrSource {
    device: Option<soapysdr::Device>,
    stream: Option<soapysdr::RxStream<Complex<f32>>>,
    freq: f64,
    rate: f64,
    channel: usize,
    lna_gain: Option<f64>,
    vga_gain: Option<f64>,
}

impl SoapySdrSource {
    pub fn new(freq: f64, rate: f64, lna_gain: Option<f64>, vga_gain: Option<f64>) -> Self {
        Self {
            device: None,
            stream: None,
            freq,
            rate,
            channel: 0,
            lna_gain,
            vga_gain,
        }
    }
}

impl SdrSource for SoapySdrSource {
    fn start(&mut self) -> Result<(), Box<dyn Error>> {
        // Enumerate devices
        let mut devices = soapysdr::enumerate("")?;
        if devices.is_empty() {
            return Err("No SoapySDR devices found".into());
        }

        println!("SDR Ingestion: Opening SoapySDR device 0");
        let dev_args = devices.remove(0);
        let dev = soapysdr::Device::new(dev_args)?;

        // Configure channel parameters
        dev.set_sample_rate(soapysdr::Direction::Rx, self.channel, self.rate)?;
        dev.set_frequency(
            soapysdr::Direction::Rx,
            self.channel,
            self.freq,
            soapysdr::Args::new(),
        )?;
        
        // Configure receiver gains for HackRF (LNA and VGA)
        let lna = self.lna_gain.unwrap_or(32.0);
        let vga = self.vga_gain.unwrap_or(30.0);
        dev.set_gain_element(soapysdr::Direction::Rx, self.channel, "LNA", lna)?;
        dev.set_gain_element(soapysdr::Direction::Rx, self.channel, "VGA", vga)?;

        // Open RX stream
        let mut stream = dev.rx_stream::<Complex<f32>>(&[self.channel])?;
        stream.activate(None)?;

        self.device = Some(dev);
        self.stream = Some(stream);

        Ok(())
    }

    fn read(&mut self, buffer: &mut [Complex<f32>]) -> Result<usize, Box<dyn Error>> {
        if let Some(ref mut stream) = self.stream {
            match stream.read(&mut [buffer], 100_000) {
                Ok(len) => Ok(len),
                Err(e) => {
                    let err_str = e.to_string();
                    if err_str.contains("Overflow") {
                        // Ignore transient overflow and return Ok(0) to skip this block and keep running
                        Ok(0)
                    } else {
                        Err(Box::new(e))
                    }
                }
            }
        } else {
            Err("Stream not active".into())
        }
    }

    fn stop(&mut self) -> Result<(), Box<dyn Error>> {
        if let Some(ref mut stream) = self.stream {
            stream.deactivate(None)?;
        }
        self.stream = None;
        self.device = None;
        Ok(())
    }

    fn set_frequency(&mut self, freq: f64) -> Result<(), Box<dyn Error>> {
        self.freq = freq;
        if let Some(ref mut dev) = self.device {
            dev.set_frequency(soapysdr::Direction::Rx, self.channel, freq, "")?;
        }
        Ok(())
    }

    fn set_sample_rate(&mut self, rate: f64) -> Result<(), Box<dyn Error>> {
        self.rate = rate;
        if let Some(ref mut dev) = self.device {
            dev.set_sample_rate(soapysdr::Direction::Rx, self.channel, rate)?;
        }
        Ok(())
    }

    fn get_sample_rate(&self) -> f64 {
        self.rate
    }

    fn get_frequency(&self) -> f64 {
        self.freq
    }
}

// =========================================================================
// 2. High-Fidelity Simulation SDR Source
// =========================================================================
pub struct SimulatedAircraft {
    // 3D position vector in meters relative to receiver (0,0,0)
    pub x: f64,
    pub y: f64,
    pub z: f64,
    // 3D velocity vector in meters/second
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    // RCS (radar cross section) scaling factor
    pub rcs: f64,
}

pub struct SimulatedTower {
    pub name: String,
    pub freq: f64,
    // 3D position vector in meters relative to receiver (0,0,0)
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub erp_watts: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct SimulatedMeteor {
    pub start_time: f64,
    pub duration: f64,
    pub initial_doppler: f64,
    pub decay_rate: f64,
    pub frequency_drift: f64,
    pub peak_amplitude: f32,
}

pub struct SimulationSdrSource {
    rate: f64,
    freq: f64,
    time: f64,
    aircraft: Vec<SimulatedAircraft>,
    towers: Vec<SimulatedTower>,
    rng: rand::rngs::StdRng,
    normal_dist: Normal<f32>,
    active: bool,
    start_time: Instant,
    meteor: Option<SimulatedMeteor>,
    speed_factor: f64,
}

impl SimulationSdrSource {
    pub fn new(freq: f64, rate: f64) -> Self {
        // Seed some mock towers
        let towers = vec![
            SimulatedTower {
                name: "WETA-FM".to_string(),
                freq: 90.9e6,
                x: -120_000.0, // 120 km West
                y: 50_000.0,   // 50 km North
                z: 350.0,      // Height 350m
                erp_watts: 75_000.0,
            },
            SimulatedTower {
                name: "WIYY-FM".to_string(),
                freq: 97.9e6,
                x: 80_000.0,  // 80 km East
                y: -90_000.0, // 90 km South
                z: 420.0,
                erp_watts: 50_000.0,
            },
            SimulatedTower {
                name: "WTOP-FM".to_string(),
                freq: 103.5e6,
                x: 20_000.0,  // 20 km East
                y: 110_000.0, // 110 km North
                z: 380.0,
                erp_watts: 100_000.0,
            },
            SimulatedTower {
                name: "WKYS-FM".to_string(),
                freq: 93.9e6,
                x: -3900.0,
                y: 5009.0,
                z: 276.0,
                erp_watts: 24_500.0,
            },
            SimulatedTower {
                name: "WHUR-FM".to_string(),
                freq: 96.3e6,
                x: 1800.0,
                y: 3260.0,
                z: 250.0,
                erp_watts: 24_000.0,
            },
        ];

        // Seed a target aircraft passing overhead
        let aircraft = vec![SimulatedAircraft {
            x: -5_000.0, // -5 km
            y: -5_000.0, // -5 km
            z: 10_500.0,  // 10.5 km altitude (34,400 feet)
            vx: 180.0,    // 180 m/s (~350 knots) East
            vy: 180.0,    // 180 m/s North
            vz: 0.0,
            rcs: 20.0, // Medium-sized aircraft
        }];

        Self {
            rate,
            freq,
            time: 0.0,
            aircraft,
            towers,
            rng: rand::rngs::StdRng::from_entropy(),
            normal_dist: Normal::new(0.0, 0.01).unwrap(), // Noise standard deviation
            active: false,
            start_time: Instant::now(),
            meteor: None,
            speed_factor: 1.0,
        }
    }

    pub fn update_aircraft_positions(&mut self, dt: f64) {
        for ac in &mut self.aircraft {
            ac.x += ac.vx * dt;
            ac.y += ac.vy * dt;
            ac.z += ac.vz * dt;

            // Loop aircraft around if they go too far (keep them in the game)
            if ac.x.abs() > 100_000.0 || ac.y.abs() > 100_000.0 {
                ac.x = -70_000.0 * ac.vx.signum();
                ac.y = -70_000.0 * ac.vy.signum();
            }
        }
    }
}

pub const C: f64 = 299_792_458.0; // Speed of light m/s

impl SdrSource for SimulationSdrSource {
    fn start(&mut self) -> Result<(), Box<dyn Error>> {
        self.active = true;
        self.start_time = Instant::now();
        self.time = 0.0;
        Ok(())
    }

    fn read(&mut self, buffer: &mut [Complex<f32>]) -> Result<usize, Box<dyn Error>> {
        if !self.active {
            return Err("Simulation not active".into());
        }

        let dt = 1.0 / self.rate;
        let num_samples = buffer.len();

        // Update positions before computing this block's physics
        self.update_aircraft_positions(dt * num_samples as f64);

        // Occasionally spawn a meteor in the simulation (e.g. 0.2% chance per block read)
        if self.meteor.is_none() && self.rng.gen_bool(0.002) {
            let duration = self.rng.gen_range(0.3..1.2);
            let initial_doppler =
                self.rng.gen_range(400.0..1800.0) * if self.rng.gen_bool(0.5) { 1.0 } else { -1.0 };
            let decay_rate = self.rng.gen_range(2.0..5.0);
            let frequency_drift = self.rng.gen_range(-400.0..400.0);
            self.meteor = Some(SimulatedMeteor {
                start_time: self.time,
                duration,
                initial_doppler,
                decay_rate,
                frequency_drift,
                peak_amplitude: self.rng.gen_range(0.02..0.05),
            });
        }

        for n in 0..num_samples {
            let t = self.time + (n as f64) * dt;

            // Baseband noise
            let n_re = self.normal_dist.sample(&mut self.rng);
            let n_im = self.normal_dist.sample(&mut self.rng);
            let mut sample = Complex::new(n_re, n_im);

            for tower in &self.towers {
                let f_offset = tower.freq - self.freq;
                if f_offset.abs() > self.rate / 2.0 {
                    continue; // Out of current tuned SDR bandwidth
                }

                let carrier_phase = 2.0 * std::f64::consts::PI * f_offset * t;

                // 1. Direct-path parameters
                let r_direct = (tower.x * tower.x + tower.y * tower.y + tower.z * tower.z).sqrt();

                // Frequency offset due to local oscillator drift (e.g. +75.0 Hz)
                let lo_offset = 75.0;
                let rx_phase_direct = 2.0 * std::f64::consts::PI * lo_offset * t;

                // Simple FM modulation: 19 kHz pilot + 1 kHz tone
                let fm_mod = 0.08 * (2.0 * std::f64::consts::PI * 19000.0 * t).sin()
                    + 0.60 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();

                // Direct path complex envelope
                let amp_direct = 1.0; // Reference level
                let total_phase_direct = carrier_phase + rx_phase_direct + fm_mod;
                let s_direct = Complex::from_polar(amp_direct as f32, total_phase_direct as f32);

                sample += s_direct;

                // 2. Reflected-paths from aircraft
                for ac in &self.aircraft {
                    let r_tx = ((ac.x - tower.x).powi(2)
                        + (ac.y - tower.y).powi(2)
                        + (ac.z - tower.z).powi(2))
                    .sqrt();
                    let r_rx = (ac.x * ac.x + ac.y * ac.y + ac.z * ac.z).sqrt();
                    let r_total = r_tx + r_rx;

                    // Bistatic delay (seconds)
                    let tau = (r_total - r_direct) / C;

                    // Bistatic range rate (velocity relative to baseline)
                    let tx_vec = [ac.x - tower.x, ac.y - tower.y, ac.z - tower.z];
                    let rx_vec = [ac.x, ac.y, ac.z];

                    let dot_tx = (ac.vx * tx_vec[0] + ac.vy * tx_vec[1] + ac.vz * tx_vec[2]) / r_tx;
                    let dot_rx = (ac.vx * rx_vec[0] + ac.vy * rx_vec[1] + ac.vz * rx_vec[2]) / r_rx;
                    let range_rate = dot_tx + dot_rx;

                    // Doppler shift (Hz) on the RF carrier
                    let lambda = C / tower.freq;
                    let doppler = -range_rate / lambda;

                    // Echo amplitude: path loss + RCS
                    let path_loss = 1.0 / (r_tx * r_rx).max(1.0);
                    let amp_reflected = (path_loss * ac.rcs * 2e7).sqrt().min(0.01) as f32; // limit to 40dB down max

                    // Reflected path complex envelope
                    let t_delayed = t - tau;
                    let fm_mod_delayed = 0.08
                        * (2.0 * std::f64::consts::PI * 19000.0 * t_delayed).sin()
                        + 0.60 * (2.0 * std::f64::consts::PI * 1000.0 * t_delayed).sin();

                    let rx_phase_reflected = 2.0 * std::f64::consts::PI * (lo_offset + doppler) * t;
                    let total_phase_reflected = carrier_phase + rx_phase_reflected + fm_mod_delayed;
                    let s_reflected =
                        Complex::from_polar(amp_reflected, total_phase_reflected as f32);

                    sample += s_reflected;
                }
            }

            // Add active meteor reflection (assumes centered around LO offset)
            if let Some(meteor) = self.meteor {
                let dt_meteor = t - meteor.start_time;
                if dt_meteor >= 0.0 && dt_meteor < meteor.duration {
                    let amp = meteor.peak_amplitude
                        * (-dt_meteor as f32 * meteor.decay_rate as f32).exp();
                    let current_doppler =
                        meteor.initial_doppler + meteor.frequency_drift * dt_meteor;
                    let phase = 2.0 * std::f64::consts::PI * (75.0 + current_doppler) * t;
                    let s_meteor = Complex::from_polar(amp, phase as f32);
                    sample += s_meteor;
                }
            }

            buffer[n] = sample;
        }

        // Clean up completed meteor
        if let Some(meteor) = self.meteor {
            if self.time > meteor.start_time + meteor.duration {
                self.meteor = None;
            }
        }

        self.time += num_samples as f64 * dt;

        // Introduce a sleep to throttle the read to real-time speed in simulation mode
        let elapsed = self.start_time.elapsed().as_secs_f64();
        let target_real_elapsed = self.time / self.speed_factor;
        if target_real_elapsed > elapsed {
            let sleep_dur = std::time::Duration::from_secs_f64(target_real_elapsed - elapsed);
            if sleep_dur.as_millis() > 0 {
                std::thread::sleep(sleep_dur.min(std::time::Duration::from_millis(50)));
            }
        }

        Ok(num_samples)
    }

    fn stop(&mut self) -> Result<(), Box<dyn Error>> {
        self.active = false;
        Ok(())
    }

    fn set_frequency(&mut self, freq: f64) -> Result<(), Box<dyn Error>> {
        self.freq = freq;
        Ok(())
    }

    fn set_sample_rate(&mut self, rate: f64) -> Result<(), Box<dyn Error>> {
        self.rate = rate;
        Ok(())
    }

    fn get_sample_rate(&self) -> f64 {
        self.rate
    }

    fn get_frequency(&self) -> f64 {
        self.freq
    }

    fn set_speed_factor(&mut self, speed: f64) -> Result<(), Box<dyn Error>> {
        self.speed_factor = speed;
        self.start_time = std::time::Instant::now() - std::time::Duration::from_secs_f64(self.time / speed);
        Ok(())
    }
}
