use crate::db::towers::latlon_to_enu;

#[derive(Clone, Debug)]
pub struct LineSegment {
    pub start: [f64; 3],
    pub end: [f64; 3],
}

pub struct OsmRailNetwork {
    pub segments: Vec<LineSegment>,
    pub ref_lat: f64,
    pub ref_lon: f64,
}

impl OsmRailNetwork {
    pub fn new() -> Self {
        Self {
            segments: Vec::new(),
            ref_lat: 0.0,
            ref_lon: 0.0,
        }
    }

    /// Fetch rail vectors from OSM Overpass API.
    pub fn fetch_rail_vectors(&mut self, lat: f64, lon: f64, radius_meters: f64) {
        self.ref_lat = lat;
        self.ref_lon = lon;
        self.segments.clear();

        let query = format!(
            "[out:json];\nway[\"railway\"=\"rail\"](around:{}, {}, {});\nout geom;",
            radius_meters, lat, lon
        );

        println!("Fetching OSM rail vectors for Train Tracking Mode...");
        let resp = match ureq::post("https://overpass-api.de/api/interpreter")
            .set("Content-Type", "application/x-www-form-urlencoded")
            .send_string(&format!("data={}", urlencoding::encode(&query))) {
                Ok(r) => r,
                Err(e) => {
                    eprintln!("Failed to fetch OSM data: {}", e);
                    return;
                }
            };

        let json: serde_json::Value = match resp.into_json() {
            Ok(j) => j,
            Err(e) => {
                eprintln!("Failed to parse OSM JSON: {}", e);
                return;
            }
        };

        if let Some(elements) = json.get("elements").and_then(|e| e.as_array()) {
            for element in elements {
                if let Some(geometry) = element.get("geometry").and_then(|g| g.as_array()) {
                    let mut prev_point: Option<[f64; 3]> = None;
                    for pt in geometry {
                        if let (Some(n_lat), Some(n_lon)) = (pt.get("lat").and_then(|v| v.as_f64()), pt.get("lon").and_then(|v| v.as_f64())) {
                            let enu = latlon_to_enu(n_lat, n_lon, 0.0, lat, lon, 0.0);
                            if let Some(prev) = prev_point {
                                self.segments.push(LineSegment {
                                    start: prev,
                                    end: enu,
                                });
                            }
                            prev_point = Some(enu);
                        }
                    }
                }
            }
        }
        println!("Fetched {} rail segments from OSM.", self.segments.len());
    }

    /// Find the closest point on the entire rail network to a given query point (x, y).
    /// Returns the (closest_pt, distance)
    pub fn closest_point(&self, px: f64, py: f64) -> Option<([f64; 3], f64)> {
        let mut min_dist_sq = f64::MAX;
        let mut best_pt = [0.0, 0.0, 0.0];

        for seg in &self.segments {
            let sx = seg.start[0];
            let sy = seg.start[1];
            let ex = seg.end[0];
            let ey = seg.end[1];

            let l2 = (ex - sx).powi(2) + (ey - sy).powi(2);
            if l2 == 0.0 {
                let d2 = (px - sx).powi(2) + (py - sy).powi(2);
                if d2 < min_dist_sq {
                    min_dist_sq = d2;
                    best_pt = [sx, sy, 0.0];
                }
                continue;
            }

            let mut t = ((px - sx) * (ex - sx) + (py - sy) * (ey - sy)) / l2;
            t = t.max(0.0).min(1.0); 

            let proj_x = sx + t * (ex - sx);
            let proj_y = sy + t * (ey - sy);

            let d2 = (px - proj_x).powi(2) + (py - proj_y).powi(2);
            if d2 < min_dist_sq {
                min_dist_sq = d2;
                best_pt = [proj_x, proj_y, 0.0]; 
            }
        }

        if min_dist_sq < f64::MAX {
            Some((best_pt, min_dist_sq.sqrt()))
        } else {
            None
        }
    }
}
