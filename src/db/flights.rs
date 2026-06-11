use crate::db::towers::latlon_to_enu;
use serde::Deserialize;
use std::sync::mpsc::Sender;
use std::sync::{Arc, RwLock};
use std::time::Duration;

#[derive(Debug, Clone)]
pub struct FlightState {
    pub icao24: String,
    pub callsign: String,
    pub lat: f64,
    pub lon: f64,
    pub altitude_m: f64,
    pub speed_mps: f64,
    pub heading_deg: f64,
}

#[derive(Deserialize, Debug)]
struct OpenSkyResponse {
    states: Option<Vec<Vec<serde_json::Value>>>,
}

pub struct FlightLookupEngine {
    ref_lat: f64,
    ref_lon: f64,
    ref_alt: f64,
    cached_flights: Arc<RwLock<Vec<FlightState>>>,
}

const EARTH_RADIUS: f64 = 6_378_137.0; // WGS-84 Earth equatorial radius (meters)

/// Converts East-North-Up (ENU) coordinates back to geodetic GPS coordinates (lat, lon, alt).
pub fn enu_to_latlon(enu: &[f64; 3], ref_lat: f64, ref_lon: f64, ref_alt: f64) -> (f64, f64, f64) {
    let east = enu[0];
    let north = enu[1];
    let up = enu[2];

    let ref_lat_rad = ref_lat.to_radians();

    let d_lat_rad = north / EARTH_RADIUS;
    let d_lon_rad = east / (EARTH_RADIUS * ref_lat_rad.cos());

    let lat = ref_lat + d_lat_rad.to_degrees();
    let lon = ref_lon + d_lon_rad.to_degrees();
    let alt = ref_alt + up;

    (lat, lon, alt)
}

impl FlightLookupEngine {
    pub fn new(
        ref_lat: f64,
        ref_lon: f64,
        ref_alt: f64,
        is_simulation: bool,
        log_tx: Sender<String>,
    ) -> Self {
        let cached_flights = Arc::new(RwLock::new(Vec::new()));

        if !is_simulation {
            let cached_flights_clone = cached_flights.clone();
            let log_tx_clone = log_tx.clone();
            std::thread::spawn(move || {
                log_tx_clone
                    .send(
                        "FlightLookup: Background thread active (polite 30s update rate)."
                            .to_string(),
                    )
                    .ok();
                loop {
                    let mut sleep_secs = 30;
                    if let Err(e) = Self::fetch_live_flights_static(
                        ref_lat,
                        ref_lon,
                        &cached_flights_clone,
                        &log_tx_clone,
                    ) {
                        log_tx_clone
                            .send(format!("FlightLookup Error: {}. Cooling off for 60s.", e))
                            .ok();
                        sleep_secs = 60;
                    }
                    std::thread::sleep(Duration::from_secs(sleep_secs));
                }
            });
        }

        Self {
            ref_lat,
            ref_lon,
            ref_alt,
            cached_flights,
        }
    }

    /// Fetch active flights in a bounding box around the receiver from OpenSky Network.
    /// Bounding box is +/- 1.5 degrees (~150 km radius).
    fn fetch_live_flights_static(
        ref_lat: f64,
        ref_lon: f64,
        cached_flights: &Arc<RwLock<Vec<FlightState>>>,
        log_tx: &Sender<String>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let la_min = ref_lat - 1.5;
        let la_max = ref_lat + 1.5;
        let lo_min = ref_lon - 1.5;
        let lo_max = ref_lon + 1.5;

        let url = format!(
            "https://opensky-network.org/api/states/all?lamin={:.4}&lomin={:.4}&lamax={:.4}&lomax={:.4}",
            la_min, lo_min, la_max, lo_max
        );

        log_tx
            .send("FlightLookup: Querying OpenSky Network API...".to_string())
            .ok();

        let response: OpenSkyResponse = serde_json::from_reader(
            ureq::get(&url)
                .timeout(Duration::from_secs(5))
                .call()?
                .into_reader(),
        )?;

        let mut flights = Vec::new();
        if let Some(states) = response.states {
            for state in states {
                if state.len() < 11 {
                    continue;
                }

                // Parse required parameters
                let icao24 = state[0].as_str().unwrap_or("").to_string();
                let callsign = state[1].as_str().unwrap_or("").trim().to_string();
                let lon = state[5].as_f64().unwrap_or(0.0);
                let lat = state[6].as_f64().unwrap_or(0.0);
                let altitude_m = state[7].as_f64().unwrap_or(0.0);
                let speed_mps = state[9].as_f64().unwrap_or(0.0);
                let heading_deg = state[10].as_f64().unwrap_or(0.0);

                if !callsign.is_empty() && lat != 0.0 && lon != 0.0 {
                    flights.push(FlightState {
                        icao24,
                        callsign,
                        lat,
                        lon,
                        altitude_m,
                        speed_mps,
                        heading_deg,
                    });
                }
            }
        }

        let num_found = flights.len();
        if let Ok(mut lock) = cached_flights.write() {
            *lock = flights;
        }
        log_tx
            .send(format!("FlightLookup: Cached {} live flights.", num_found))
            .ok();
        Ok(())
    }

    /// Match a target EKF spatial state [x, y, z, vx, vy, vz] to the nearest flight in the database.
    /// Returns the matched flight number/callsign and description if found.
    pub fn match_flight(&self, state: &[f64; 6], is_simulation: bool) -> Option<String> {
        if is_simulation {
            // In simulation, we mock match the simulated flight target
            let alt = state[2];
            let vx = state[3];
            let vy = state[4];
            let speed = (vx * vx + vy * vy).sqrt();
            if alt > 8000.0 && speed > 200.0 {
                return Some("AAL191 (B788)".to_string());
            }
            return Some("N420SP (C172)".to_string());
        }

        let flights_guard = self.cached_flights.read().ok()?;
        if flights_guard.is_empty() {
            return None;
        }

        // 2. Convert EKF coordinates (ENU) to Lat/Lon GPS coordinates
        let (_t_lat, _t_lon, t_alt) = enu_to_latlon(
            &[state[0], state[1], state[2]],
            self.ref_lat,
            self.ref_lon,
            self.ref_alt,
        );

        let vx = state[3];
        let vy = state[4];
        let vz = state[5];
        let t_speed = (vx * vx + vy * vy + vz * vz).sqrt();

        let mut best_match = None;
        let mut best_score = 3.0; // matching score gate threshold

        // 3. Scan cached flights for nearest match
        for flight in flights_guard.iter() {
            // Coordinate distance in km (using flat-Earth approximation since bounding box is small)
            let enu_coord = latlon_to_enu(
                flight.lat,
                flight.lon,
                flight.altitude_m,
                self.ref_lat,
                self.ref_lon,
                self.ref_alt,
            );
            let dx = (state[0] - enu_coord[0]) / 1000.0;
            let dy = (state[1] - enu_coord[1]) / 1000.0;
            let dz = (state[2] - enu_coord[2]) / 1000.0;
            let dist_km = (dx * dx + dy * dy + dz * dz).sqrt();

            let speed_diff = (t_speed - flight.speed_mps).abs();
            let alt_diff = (t_alt - flight.altitude_m).abs();

            // Match metric combines position (gate < 15km), altitude (gate < 2500m), and speed (gate < 60m/s)
            let score = dist_km / 10.0 + alt_diff / 1500.0 + speed_diff / 40.0;

            if score < best_score {
                best_score = score;
                best_match = Some(format!(
                    "{} ({} - {:.0}m)",
                    flight.callsign, flight.icao24, flight.altitude_m
                ));
            }
        }

        best_match
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_enu_to_latlon_roundtrip() {
        let ref_lat = 38.8951;
        let ref_lon = -77.0364;
        let ref_alt = 10.0;

        // Convert a point 10km East, 20km North, 5km Up
        let enu = [10_000.0, 20_000.0, 5000.0];
        let (lat, lon, alt) = enu_to_latlon(&enu, ref_lat, ref_lon, ref_alt);

        // Convert back using latlon_to_enu
        let enu_back = latlon_to_enu(lat, lon, alt, ref_lat, ref_lon, ref_alt);

        assert!(
            (enu[0] - enu_back[0]).abs() < 1e-3,
            "East coordinate mismatch"
        );
        assert!(
            (enu[1] - enu_back[1]).abs() < 1e-3,
            "North coordinate mismatch"
        );
        assert!(
            (enu[2] - enu_back[2]).abs() < 1e-3,
            "Up coordinate mismatch"
        );
    }

    #[test]
    fn test_flight_matcher_simulation() {
        let (log_tx, _log_rx) = std::sync::mpsc::channel();
        let engine = FlightLookupEngine::new(38.8951, -77.0364, 10.0, true, log_tx);

        // Simulated High Alt Target
        let high_alt_state = [1000.0, 2000.0, 9000.0, 210.0, 150.0, 0.0];
        let match1 = engine.match_flight(&high_alt_state, true);
        assert_eq!(match1, Some("AAL191 (B788)".to_string()));

        // Simulated Low Alt Target
        let low_alt_state = [1000.0, 2000.0, 1000.0, 30.0, 40.0, 0.0];
        let match2 = engine.match_flight(&low_alt_state, true);
        assert_eq!(match2, Some("N420SP (C172)".to_string()));
    }

    #[test]
    fn test_flight_matcher_live_cached() {
        let (log_tx, _log_rx) = std::sync::mpsc::channel();
        let engine = FlightLookupEngine::new(38.8951, -77.0364, 10.0, true, log_tx);

        // Manually seed the cached flights list
        {
            let mut lock = engine.cached_flights.write().unwrap();
            lock.push(FlightState {
                icao24: "a8b9c0".to_string(),
                callsign: "UAL123".to_string(),
                lat: 38.9951,  // ~11.1 km north of reference
                lon: -77.0364, // same longitude
                altitude_m: 3000.0,
                speed_mps: 180.0,
                heading_deg: 90.0,
            });
        }

        // Test matching close target
        // ENU coordinate for flight is roughly [0.0, 11112.0, 2990.0]
        // Target state has velocity [150.0, 100.0, 0.0] (speed = 180 m/s)
        let close_state = [0.0, 11112.0, 2990.0, 150.0, 100.0, 0.0];
        let matched = engine.match_flight(&close_state, false);
        assert!(matched.is_some(), "Should match close target");
        assert_eq!(matched.unwrap(), "UAL123 (a8b9c0 - 3000m)");

        // Test matching far target (should exceed gate score limit of 3.0)
        let far_state = [50_000.0, 50_000.0, 10_000.0, 350.0, 0.0, 0.0];
        let matched_far = engine.match_flight(&far_state, false);
        assert!(matched_far.is_none(), "Should NOT match far target");
    }
}
