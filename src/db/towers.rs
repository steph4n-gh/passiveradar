use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::{Read, Write};
use std::path::Path;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransmitterTower {
    pub name: String,
    pub callsign: String,
    pub frequency_hz: f64,
    pub latitude: f64,
    pub longitude: f64,
    pub elevation_m: f64,
    pub erp_watts: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReceiverConfig {
    pub latitude: f64,
    pub longitude: f64,
    pub elevation_m: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TowerDatabase {
    pub receiver: ReceiverConfig,
    pub towers: Vec<TransmitterTower>,
}

const EARTH_RADIUS: f64 = 6_378_137.0; // WGS-84 Earth equatorial radius (meters)

/// Convert GPS coordinates (lat, lon, alt) to East-North-Up (ENU) Cartesian coordinates
/// relative to a reference receiver position.
pub fn latlon_to_enu(
    lat: f64,
    lon: f64,
    alt: f64,
    ref_lat: f64,
    ref_lon: f64,
    ref_alt: f64,
) -> [f64; 3] {
    let lat_rad = lat.to_radians();
    let lon_rad = lon.to_radians();
    let ref_lat_rad = ref_lat.to_radians();
    let ref_lon_rad = ref_lon.to_radians();

    let d_lat = lat_rad - ref_lat_rad;
    let d_lon = lon_rad - ref_lon_rad;

    // Easting
    let east = EARTH_RADIUS * d_lon * ref_lat_rad.cos();
    // Northing
    let north = EARTH_RADIUS * d_lat;
    // Up
    let up = alt - ref_alt;

    [east, north, up]
}

impl TowerDatabase {
    /// Load database from a JSON file, or create a default database with FM towers in the Washington DC area.
    pub fn load_or_create<P: AsRef<Path>>(path: P) -> Result<Self, Box<dyn std::error::Error>> {
        let path_ref = path.as_ref();
        if path_ref.exists() {
            let mut file = File::open(path_ref)?;
            let mut contents = String::new();
            file.read_to_string(&mut contents)?;
            let db: TowerDatabase = serde_json::from_str(&contents)?;
            Ok(db)
        } else {
            // Default receiver config (located in Washington, DC area)
            let receiver = ReceiverConfig {
                latitude: 38.8951,
                longitude: -77.0364,
                elevation_m: 10.0,
            };

            // Seeding default high-power FM towers in the MD/VA/DC area
            let towers = vec![
                TransmitterTower {
                    name: "WETA-FM".to_string(),
                    callsign: "WETA".to_string(),
                    frequency_hz: 90.9e6,
                    latitude: 38.9615,
                    longitude: -77.1518,
                    elevation_m: 350.0,
                    erp_watts: 75_000.0,
                },
                TransmitterTower {
                    name: "WIYY-FM (98 Rock)".to_string(),
                    callsign: "WIYY".to_string(),
                    frequency_hz: 97.9e6,
                    latitude: 39.3512,
                    longitude: -76.6341,
                    elevation_m: 420.0,
                    erp_watts: 50_000.0,
                },
                TransmitterTower {
                    name: "WTOP-FM".to_string(),
                    callsign: "WTOP".to_string(),
                    frequency_hz: 103.5e6,
                    latitude: 39.0435,
                    longitude: -77.2014,
                    elevation_m: 380.0,
                    erp_watts: 100_000.0,
                },
                TransmitterTower {
                    name: "WKYS-FM".to_string(),
                    callsign: "WKYS".to_string(),
                    frequency_hz: 93.9e6,
                    latitude: 38.9401,
                    longitude: -77.0814,
                    elevation_m: 286.0,
                    erp_watts: 24_500.0,
                },
                TransmitterTower {
                    name: "WHUR-FM".to_string(),
                    callsign: "WHUR".to_string(),
                    frequency_hz: 96.3e6,
                    latitude: 38.9244,
                    longitude: -77.0156,
                    elevation_m: 260.0,
                    erp_watts: 24_000.0,
                },
            ];

            let db = TowerDatabase { receiver, towers };

            // Serialize to file
            let serialized = serde_json::to_string_pretty(&db)?;
            let mut file = File::create(path_ref)?;
            file.write_all(serialized.as_bytes())?;

            Ok(db)
        }
    }

    /// Lookup a transmitter by frequency (matching within 100 kHz).
    pub fn lookup_tower(&self, freq_hz: f64) -> Option<&TransmitterTower> {
        self.towers
            .iter()
            .find(|t| (t.frequency_hz - freq_hz).abs() < 100e3)
    }

    /// Get the Cartesian ENU coordinates of a transmitter relative to the receiver.
    pub fn get_tower_enu(&self, tower: &TransmitterTower) -> [f64; 3] {
        latlon_to_enu(
            tower.latitude,
            tower.longitude,
            tower.elevation_m,
            self.receiver.latitude,
            self.receiver.longitude,
            self.receiver.elevation_m,
        )
    }

    /// Find the mathematically optimal center frequency to tune to in order to maximize target tracking potential.
    /// Returns the optimal center frequency (in Hz) and the list of towers covered by that window.
    pub fn find_optimal_tuning(&self, rate_hz: f64) -> (f64, Vec<TransmitterTower>) {
        if self.towers.is_empty() {
            return (90.9e6, Vec::new());
        }

        // 1. Sort a copy of towers by frequency
        let mut sorted_towers = self.towers.clone();
        sorted_towers.sort_by(|a, b| a.frequency_hz.partial_cmp(&b.frequency_hz).unwrap());

        // 2. Pre-calculate quality scores and minimum distance for each tower
        // Q = erp_watts / distance^2 (using 3D distance relative to receiver)
        let mut tower_details = Vec::new();
        for tower in &sorted_towers {
            let enu = self.get_tower_enu(tower);
            let dist_sq = enu[0] * enu[0] + enu[1] * enu[1] + enu[2] * enu[2];
            let dist = dist_sq.sqrt().max(1.0); // prevent division by zero
            let erp = tower.erp_watts.max(1.0);
            let quality = erp / dist_sq.max(1.0);
            tower_details.push((tower.clone(), quality, dist));
        }

        let mut best_center = sorted_towers[0].frequency_hz;
        let mut best_score = -1.0;
        let mut best_towers = Vec::new();
        let mut best_min_dist = f64::MAX;

        // 3. Exhaustive search over all subsets of towers fitting inside the rate_hz bandwidth
        for i in 0..sorted_towers.len() {
            for j in i..sorted_towers.len() {
                let f_min = sorted_towers[i].frequency_hz;
                let f_max = sorted_towers[j].frequency_hz;

                if f_max - f_min > rate_hz {
                    break; // Subsequence span exceeds the sample rate bandwidth
                }

                // Candidate center frequency
                let f_c = (f_min + f_max) / 2.0;

                // Evaluate all towers in the database against this candidate window
                let mut current_towers = Vec::new();
                let mut current_score = 0.0;
                let mut current_min_dist = f64::MAX;

                for (tower, q, dist) in &tower_details {
                    let offset = (tower.frequency_hz - f_c).abs();
                    if offset <= rate_hz / 2.0 {
                        current_towers.push(tower.clone());
                        current_score += q;
                        if *dist < current_min_dist {
                            current_min_dist = *dist;
                        }
                    }
                }

                // We want to maximize:
                // 1. Total quality score of covered towers
                // 2. Tie-breaker: Minimize distance of the closest tower covered
                if current_score > best_score
                    || ((current_score - best_score).abs() < 1e-9 && current_min_dist < best_min_dist)
                {
                    best_score = current_score;
                    best_center = f_c;
                    best_towers = current_towers;
                    best_min_dist = current_min_dist;
                }
            }
        }

        (best_center, best_towers)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_optimal_tuning_selection() {
        // Create a mock TowerDatabase
        let receiver = ReceiverConfig {
            latitude: 38.8951,
            longitude: -77.0364,
            elevation_m: 10.0,
        };

        // Seed 3 towers
        // WETA: 90.9 MHz
        // WKYS: 93.9 MHz
        // WHUR: 96.3 MHz
        let towers = vec![
            TransmitterTower {
                name: "WETA-FM".to_string(),
                callsign: "WETA".to_string(),
                frequency_hz: 90.9e6,
                latitude: 38.9615,
                longitude: -77.1518,
                elevation_m: 350.0,
                erp_watts: 75_000.0,
            },
            TransmitterTower {
                name: "WKYS-FM".to_string(),
                callsign: "WKYS".to_string(),
                frequency_hz: 93.9e6,
                latitude: 38.9401,
                longitude: -77.0814,
                elevation_m: 286.0,
                erp_watts: 24_500.0,
            },
            TransmitterTower {
                name: "WHUR-FM".to_string(),
                callsign: "WHUR".to_string(),
                frequency_hz: 96.3e6,
                latitude: 38.9244,
                longitude: -77.0156,
                elevation_m: 260.0,
                erp_watts: 24_000.0,
            },
        ];

        let db = TowerDatabase { receiver, towers };

        // Test 1: Low sample rate (e.g. 2.0 MHz).
        // Since no two towers can fit inside a 2.0 MHz bandwidth,
        // it should pick the single highest-quality tower (WHUR-FM, because it is closest at 3.7 km).
        let (freq1, towers1) = db.find_optimal_tuning(2.0e6);
        assert_eq!(towers1.len(), 1);
        assert_eq!(towers1[0].name, "WHUR-FM");
        assert_eq!(freq1, 96.3e6);

        // Test 2: Wider sample rate (e.g. 6.0 MHz).
        // WETA (90.9) and WHUR (96.3) span 5.4 MHz.
        // It should pick a center frequency that covers all 3 towers.
        // Center of 90.9 and 96.3 is 93.6 MHz.
        let (freq2, towers2) = db.find_optimal_tuning(6.0e6);
        assert_eq!(towers2.len(), 3);
        assert_eq!(freq2, 93.6e6);
    }
}


