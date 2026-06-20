# Passive Radar Practical Operations Guide

Welcome to the DIY setup and operations guide! This document explains how to get your passive radar station up and running with minimal budget and maximum performance—using a vertical whip antenna, a kitchen baking sheet, and a HackRF or RTL-SDR to track aircraft using ambient FM radio broadcasts.

---

## 1. Physical Antenna Setup

Passive radar is a "do more with less" technology. Instead of transmitting energy, we listen to ambient FM radio signals (88–108 MHz) bouncing off airplanes (forward scatter). To do this effectively, you need a proper antenna system.

### Why a Ground Plane?
A standard quarter-wave vertical whip antenna expects a ground plane underneath it to act as an electromagnetic mirror. This mirror creates an "image antenna" counterpart, transforming the quarter-wave whip into a virtual half-wave dipole. Without a ground plane, the antenna has poor impedance matching, high noise, and low gain.

### Step-by-Step DIY Assembly
1. **Find a Baking Sheet**: Grab a clean, uninsulated metallic kitchen baking sheet (steel or aluminum works great; do not use non-conductive glass or ceramic!).
2. **Mount the Antenna**: Place the magnetic base of your telescoping whip antenna directly in the center of the baking sheet. The magnet provides structural stability and capacitive coupling to the ground plane.
3. **Extend to Quarter-Wave Length**: Telescoping antennas can be tuned by adjusting their physical length. For the FM broadcast band, calculate the optimal length:
   $$
   L = \frac{c}{4 \cdot f}
   $$
   For the center of the FM band ($\sim 98 \text{ MHz}$):
   $$
   L = \frac{3 \times 10^8 \text{ m/s}}{4 \cdot 98 \times 10^6 \text{ Hz}} \approx 0.765 \text{ meters } (30.1 \text{ inches})
   $$
   Extend your whip antenna to approximately **76 cm (30 inches)**.
4. **Positioning**: 
   * Place the setup close to a window with a clear view of the sky or towards the flight paths/towers you want to monitor.
   * Keep it away from large metal objects, power lines, and noisy computers which can block RF signals or introduce electromagnetic interference (EMI).

---

## 2. FM Tuning and SDR Optimization

To detect aircraft, the system needs a reference FM signal (direct path from the transmitter) and a surveillance channel (scattered path from the aircraft). When using a single HackRF or RTL-SDR, we tune to a strong local FM transmitter.

### Choosing Reference Transmitters
Select high-power local commercial FM radio stations (between 88.0 and 108.0 MHz). Look for stations that:
* Have a known transmitter tower location (you can find these in local databases or FCC registries).
* Have a strong, stable signal at your location.
* Are located far enough away from your receiver to form a good bistatic triangle (transmitter $\to$ aircraft $\to$ receiver).

### Tuning Center Frequency and Offset
To calibrate your SDR's local oscillator and align with the peak power spectrum, use the command-line flags. You can pass the specific FM frequency in MHz:
```bash
# Tune to 98.1 MHz with a 2.048 MSPS sample rate
cargo run --release -- --mode sdr --freq 98.1 --rate 2.048
```
The pipeline automatically designs a Digital Down Converter (DDC) to shift the target FM carrier to baseband and filter it down to a narrowband channel.

### Optimizing LNA and VGA Gains
Because forward scatter signal reflections from aircraft are extremely weak, gain optimization is critical:
* **LNA (Low Noise Amplifier) Gain**: The first stage of amplification. The default is `32.0` dB. If signals are weak, increase this value to amplify reflections. If you are close to the transmitter, decrease it to prevent ADC saturation.
* **VGA (Variable Gain Amplifier) Gain**: The second stage of amplification. The default is `30.0` dB. It adjusts baseband signal levels before digitizing.
```bash
# Custom gains optimization
cargo run --release -- --mode sdr --freq 98.1 --lna 38.0 --vga 24.0
```
*Note: If you notice the noise floor rising dramatically or see fake periodic spikes across the spectrum, your gains are too high, causing intermodulation distortion (spurs).*

---

## 3. Troubleshooting Q&A

| Issue | Potential Cause | Solution / Troubleshooting Step |
| :--- | :--- | :--- |
| **Websocket Connection Timeout / Web HUD disconnected** | Host or Port conflicts, or firewall blocking the connection. | Ensure the WebSocket listener port is open. Use the `--port` flag to assign a custom WebSocket port, or `--web-port` for the web UI companion. Ensure host matches (e.g., `--host 127.0.0.1` or `--host 0.0.0.0` for external devices). |
| **Buffer Overflows / "SDR Buffer Overflow" logs** | The CPU cannot process incoming SDR samples fast enough. | 1. Run the application in release mode (`cargo run --release`).<br>2. Disable GPU acceleration using `--disable-gpu` to bypass WebGPU pipeline bottlenecks.<br>3. Lower the sample rate using `--rate 1.0` or `--rate 2.048` (do not go below 1.0 MSPS to maintain channel definition). |
| **Target Ghosting / Rapid target pruning** | Spurious intersections are being identified, or legitimate tracks are marked as "ghosts" and deleted. | The Čech Cohomology filter prunes targets whose computed spatial-Doppler discrepancy exceeds 300.0 Hz. Verify that:<br>1. Tower positions are correct.<br>2. Your receiver's latitude/longitude are set accurately via `--lat` and `--lon`.<br>3. Adjust the alignment compass heading using `--heading <deg>` to match your antenna's physical orientation. |
| **GPU FFT errors / WebGPU initialization failure** | Unsupported graphics driver or lack of GPU compute compatibility. | The system defaults to GPU acceleration for the heavy FFT processing steps. Run with the `--disable-gpu` flag to fall back to the highly optimized RustFFT CPU backend. |
| **SoapySdr source loading error / SDR device not found** | Missing driver libraries, loose USB connections, or device contention. | 1. Unplug and replug the USB cable.<br>2. Verify SoapySDR is detecting the device by running `SoapySDRUtil --find` in your terminal.<br>3. Make sure no other SDR application (like GQRX or SDR#) is using the device. |
| **Spectrum looks completely flat or only contains noise** | Incorrect frequency, lack of ground plane, or loose antenna connection. | 1. Double check the telescoping length of your whip antenna (approx 76 cm).<br>2. Verify that the antenna is connected securely to the SDR's RF input port.<br>3. Try a different FM station frequency using `--freq <val>`. |

---

## Related Documents
*   For the mathematical formulas, equations, and DSP algorithms utilized in the pipeline, see [DSP and Mathematical Foundations (math.md)](math.md).

