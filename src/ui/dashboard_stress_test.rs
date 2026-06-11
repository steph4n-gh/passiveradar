use crate::ui::dashboard::Dashboard;
use ratatui::layout::Rect;
use ratatui::backend::TestBackend;
use ratatui::Terminal;
use std::time::{Instant, Duration};

#[test]
fn test_dashboard_stress_and_cache() {
    let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string());
    
    // Fill history
    for _ in 0..200 {
        db.add_spectrum(vec![0.5; 256]);
    }
    db.update_caf(vec![vec![0.5; 256]; 200]);

    let backend = TestBackend::new(200, 200);
    let mut terminal = Terminal::new(backend).unwrap();

    let start = Instant::now();
    for i in 0..1000 {
        // Pretend we are rendering many frames in quick succession
        terminal.draw(|f| {
            let size = f.size();
            db.render(f, size, &[], &[]);
        }).unwrap();
    }
    let elapsed = start.elapsed();
    println!("1000 renders took {:?}", elapsed);
}
