use passiveradar::ui::dashboard::Dashboard;
use ratatui::backend::TestBackend;
use ratatui::Terminal;
use ratatui::layout::Rect;
use std::time::Instant;

#[test]
fn test_profile() {
    let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string());
    for _ in 0..200 {
        db.add_spectrum(vec![0.0; 1000]);
    }
    let area = Rect::new(0, 0, 300, 300);
    
    let t0 = Instant::now();
    // Simulate one update
    let mut backend = TestBackend::new(300, 300);
    let mut terminal = Terminal::new(backend).unwrap();
    terminal.draw(|f| {
        db.render(f, area, &[], &[]);
    }).unwrap();
    println!("First render (build): {:?}", t0.elapsed());
    
    let t1 = Instant::now();
    terminal.draw(|f| {
        db.render(f, area, &[], &[]);
    }).unwrap();
    println!("Second render (cached): {:?}", t1.elapsed());
}

#[test]
fn test_clone_speed() {
    let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string());
    for _ in 0..200 {
        db.add_spectrum(vec![0.0; 1000]);
    }
    let area = Rect::new(0, 0, 300, 300);
    
    let mut backend = TestBackend::new(300, 300);
    let mut terminal = Terminal::new(backend).unwrap();
    terminal.draw(|f| {
        db.render(f, area, &[], &[]);
    }).unwrap();

    let t2 = Instant::now();
    for _ in 0..100 {
        let _cloned = db.cached_waterfall_lines.clone();
    }
    println!("100 clones took: {:?}", t2.elapsed());
}

#[test]
fn test_draw_speed() {
    let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string());
    for _ in 0..200 {
        db.add_spectrum(vec![0.0; 1000]);
    }
    let area = Rect::new(0, 0, 300, 300);
    
    let mut backend = TestBackend::new(300, 300);
    let mut terminal = Terminal::new(backend).unwrap();
    terminal.draw(|f| {
        db.render(f, area, &[], &[]);
    }).unwrap();

    let t2 = Instant::now();
    for _ in 0..100 {
        terminal.draw(|f| {
            // just empty render
            use ratatui::widgets::{Block, Paragraph};
            use ratatui::text::Text;
            f.render_widget(Paragraph::new(db.cached_waterfall_lines.clone()), area);
        }).unwrap();
    }
    println!("100 empty draws took: {:?}", t2.elapsed());
}

#[test]
fn test_span_count() {
    let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string());
    for _ in 0..200 {
        db.add_spectrum(vec![0.0; 1000]);
    }
    let area = Rect::new(0, 0, 300, 300);
    
    let mut backend = TestBackend::new(300, 300);
    let mut terminal = Terminal::new(backend).unwrap();
    terminal.draw(|f| {
        db.render(f, area, &[], &[]);
    }).unwrap();

    let mut total_spans = 0;
    for line in &db.cached_waterfall_lines {
        total_spans += line.spans.len();
    }
    println!("Total spans for 200 rows: {}", total_spans);
}
