use passiveradar::ui::dashboard::Dashboard;
use ratatui::backend::TestBackend;
use ratatui::Terminal;
use ratatui::layout::Rect;
use passiveradar::tracking::bank::{TrackedTarget, TrackState};
use std::time::Instant;

#[test]
fn test_stress_render() {
    let backend = TestBackend::new(300, 300);
    let mut terminal = Terminal::new(backend).unwrap();
    let mut dashboard = Dashboard::new(100e6, 2.4e6, 0.0, "SDR".into());

    let mut targets = Vec::new();
    for i in 0..100 {
        let mut t = TrackedTarget {
            id: i,
            ekf: passiveradar::tracking::ekf::BistaticEkf::new([0.0; 6], 1.0, 1.0, 1.0),
            state: TrackState::Active,
            hits: 1,
            misses: 0,
            coasting_frames: 0,
            history: vec![],
            classification: "Unknown".into(),
            terminated_at: None,
            start_time: Instant::now(),
            fingerprint_history: vec![],
            jem: passiveradar::tracking::jem::JemAnalyzer::new(),
            tracking_towers: vec![],
        };
        targets.push(t);
    }

    let start = Instant::now();
    for i in 0..1000 {
        dashboard.data_version += 1;
        dashboard.selected_target_id = Some((i % 100) as u32);
        terminal.draw(|f| {
            dashboard.render(f, Rect::new(0, 0, 300, 300), &targets, &[]);
        }).unwrap();
    }
    println!("Stress test took: {:?}", start.elapsed());
}
