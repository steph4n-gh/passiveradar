use passiveradar::ui::dashboard::Dashboard;
use ratatui::backend::TestBackend;
use ratatui::Terminal;
use ratatui::layout::Rect;
use std::time::Instant;

#[test]
fn test_waterfall_stress_and_caching() {
    let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string());
    
    // Simulate lots of history and CAF matrices
    for _ in 0..200 {
        db.add_spectrum(vec![0.0; 1000]);
    }
    
    let backend = TestBackend::new(300, 300);
    let mut terminal = Terminal::new(backend).unwrap();

    let area = Rect::new(0, 0, 300, 300);
    let targets = vec![];
    let transients = vec![];
    
    // Render once to populate cache
    let _t = Instant::now(); terminal.draw(|f| {
        db.render(f, area, &targets, &transients);
    }).unwrap(); println!("draw: {:?}", _t.elapsed());

    let cached_waterfall_len = db.cached_waterfall_lines.len();
    assert!(cached_waterfall_len > 0, "Waterfall lines should be cached");

    // Stress test: render 100 times in quick succession.
    // Because of caching (100ms threshold), it shouldn't rebuild the lines.
    // If it rebuilds the lines constantly, it might be slow or leak.
    let start = Instant::now();
    for _ in 0..100 {
        let _t = Instant::now(); terminal.draw(|f| {
            db.render(f, area, &targets, &transients);
        }).unwrap(); println!("draw: {:?}", _t.elapsed());
    }
    let elapsed = start.elapsed();
    
    // Since it's cached, 100 renders should be very fast.
    println!("100 cached renders took: {:?}", elapsed);
    assert!(elapsed.as_millis() < 15000, "Render is too slow, caching might be broken");
    
    // Test coalescing logic: Add a spectrum with alternating colors to defeat coalescing,
    // then one with solid colors to verify coalescing.
    let mut solid_spectrum = vec![0.0; 1000];
    for i in 0..1000 {
        solid_spectrum[i] = 10.0; // High SNR -> all same color
    }
    db.caf_matrix = vec![solid_spectrum.clone(); 200];
    
    // Force update
    db.cached_waterfall_area = Rect::new(0,0,0,0);
    let _t = Instant::now(); terminal.draw(|f| {
        db.render(f, area, &targets, &transients);
    }).unwrap(); println!("draw: {:?}", _t.elapsed());
    
    let solid_spans_count = db.cached_waterfall_lines.last().unwrap().spans.len();
    
    let mut alternating_spectrum = vec![0.0; 1000];
    for i in 0..1000 {
        if i % 2 == 0 {
            alternating_spectrum[i] = 10.0; // High SNR
        } else {
            alternating_spectrum[i] = 0.5; // Low SNR
        }
    }
    db.caf_matrix = vec![alternating_spectrum.clone(); 200];
    
    // Force update
    db.cached_waterfall_area = Rect::new(0,0,0,0);
    let _t = Instant::now(); terminal.draw(|f| {
        db.render(f, area, &targets, &transients);
    }).unwrap(); println!("draw: {:?}", _t.elapsed());

    let alt_spans_count = db.cached_waterfall_lines.last().unwrap().spans.len();
    
    println!("Solid spans count: {}\nSolid spans: {:#?}", solid_spans_count, db.cached_waterfall_lines.last().unwrap().spans);
    println!("Alternating spans count: {}", alt_spans_count);
    
    assert!(solid_spans_count < alt_spans_count, "Coalescing failed to reduce span count");
    
    // Check if there are memory issues by rendering very large area
    let backend_large = TestBackend::new(1000, 1000);
    let mut terminal_large = Terminal::new(backend_large).unwrap();
    let area_large = Rect::new(0, 0, 1000, 1000);
    db.cached_waterfall_area = Rect::new(0,0,0,0);
    
    terminal_large.draw(|f| {
        db.render(f, area_large, &targets, &transients);
    }).unwrap(); println!("draw: {:?}", _t.elapsed());
    
    println!("Stress test completed successfully");
}
