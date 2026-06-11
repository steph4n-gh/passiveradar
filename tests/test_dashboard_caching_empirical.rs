use passiveradar::ui::dashboard::Dashboard;
use ratatui::backend::TestBackend;
use ratatui::Terminal;
use std::time::{Duration, Instant};

#[test]
fn test_dashboard_cache_stress_and_leak() {
    let mut db = Dashboard::new(100e6, 2.4e6, 0.0, "HackRF".to_string());
    
    // Add some logs
    db.add_log("Startup log 1".to_string());
    db.add_log("Startup log 2".to_string());

    // Targets
    let mut targets = vec![];
    let target = passiveradar::tracking::bank::TrackedTarget {
        id: 42,
        ekf: passiveradar::tracking::ekf::BistaticEkf::new([50.0, 2000.0, 3000.0, 50.0, 0.0, 0.0], 100.0, 10.0, 1.0),
        state: passiveradar::tracking::bank::TrackState::Active,
        hits: 5,
        misses: 0,
        history: vec![[0.0; 6]],
        classification: "Drone".to_string(),
        terminated_at: None,
        coasting_frames: 0,
        jem: passiveradar::tracking::jem::JemAnalyzer::new(),
        start_time: std::time::Instant::now(),
        fingerprint_history: vec![],
        tracking_towers: vec![],
    };
    targets.push(target);

    // Transients
    let transients = vec![];

    let backend = TestBackend::new(200, 100);
    let mut terminal = Terminal::new(backend).unwrap();

    let mut render_count = 0;
    let mut table_update_count = 0;
    let mut prev_table_update = None;
    
    let start_time = Instant::now();

    // Loop 50 times as fast as possible to verify it caches, without sleep
    for _ in 0..50 {
        terminal.draw(|f| {
            let size = f.size();
            db.render(f, size, &targets, &transients);
        }).unwrap();
        
        if db.last_table_update != prev_table_update {
            table_update_count += 1;
            prev_table_update = db.last_table_update;
        }

        render_count += 1;
    }

    let elapsed = start_time.elapsed();
    let elapsed_ms = elapsed.as_millis();
    let expected_max_updates = (elapsed_ms / 300) + 2;

    println!("Render count: {}", render_count);
    println!("Elapsed time: {} ms", elapsed_ms);
    println!("Table update count: {} (expected <= {})", table_update_count, expected_max_updates);
    
    assert!(table_update_count <= expected_max_updates as usize, "Caching failed to prevent rapid updates! Updates: {} in {}ms", table_update_count, elapsed_ms);
    
    assert!(db.cached_table_rows.len() <= 10, "Table cache is leaking rows: {}", db.cached_table_rows.len());
    // Since there's 1 log array of 2 elements, we'd expect log lines to not blow up
    assert!(db.cached_logs_lines.len() <= 10, "Logs cache is leaking lines: {}", db.cached_logs_lines.len());
}
