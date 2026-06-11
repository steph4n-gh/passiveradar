#[test]
fn test_math() {
    let mut alt = 0;
    let mut prev_even = true;
    for w in 0..137 {
        let bin_idx = 350 + (w * 300) / 137;
        let is_even = bin_idx % 2 == 0;
        if is_even != prev_even {
            alt += 1;
        }
        prev_even = is_even;
    }
    println!("Alternations: {}", alt);
}
