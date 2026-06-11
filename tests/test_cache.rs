use passiveradar::ui::dashboard::Dashboard;
use ratatui::backend::TestBackend;
use ratatui::Terminal;
use std::time::Instant;

fn main() {
    let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string());
    db.update_caf(vec![vec![0.5; 256]; 200]);

    let backend = TestBackend::new(200, 200);
    let mut terminal = Terminal::new(backend).unwrap();

    terminal.draw(|f| {
        let size = f.size();
        db.render(f, size, &[], &[]);
    }).unwrap();
    
    let initial_last_update = db.last_waterfall_update;
    assert!(initial_last_update.is_some());

    for _ in 0..1000 {
        terminal.draw(|f| {
            let size = f.size();
            db.render(f, size, &[], &[]);
        }).unwrap();
    }
    assert_eq!(db.last_waterfall_update, initial_last_update);
    println!("Test passed!");
}
