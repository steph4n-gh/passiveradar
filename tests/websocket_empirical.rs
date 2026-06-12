use std::process::{Command, Stdio, Child};
use std::thread;
use std::time::Duration;
use tungstenite::{connect, Message};

struct KillOnDrop(Child);
impl Drop for KillOnDrop {
    fn drop(&mut self) {
        let _ = self.0.kill();
    }
}

#[test]
fn test_websocket_server_empirical() {
    // Find a free TCP port dynamically to prevent port collisions
    let port = {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("Failed to bind dynamic port");
        listener.local_addr().unwrap().port()
    };

    // 2. Start the passiveradar binary in background in test mode with our test script
    let bin_path = env!("CARGO_BIN_EXE_passiveradar");
    let log_file = std::fs::File::create("child_error_log.txt").unwrap();
    let child = Command::new(bin_path)
        .arg("--mode")
        .arg("sim")
        .arg("--test-script")
        .arg("tests/test_ws_script.txt")
        .arg("--port")
        .arg(port.to_string())
        .env("RUST_BACKTRACE", "1")
        .stdin(Stdio::null())
        .stdout(Stdio::from(log_file.try_clone().unwrap()))
        .stderr(Stdio::from(log_file))
        .spawn()
        .expect("Failed to spawn passiveradar binary");
    
    let mut child_guard = KillOnDrop(child);
    
    // Give the server a moment to start up and bind to the port
    thread::sleep(Duration::from_millis(1500));
    
    // Check if child process is still running
    if let Ok(Some(status)) = child_guard.0.try_wait() {
        panic!("Child process exited early before connection: {:?}", status);
    }

    let ws_url = format!("ws://127.0.0.1:{}", port);

    // 3. Connect client 1
    println!("Connecting Client 1 to {}...", ws_url);
    let (mut socket1, response1) = connect(&ws_url)
        .expect("Failed to connect Client 1");
    println!("Client 1 connected. Response status: {}", response1.status());
    
    // 4. Connect client 2
    println!("Connecting Client 2 to {}...", ws_url);
    let (mut socket2, response2) = connect(&ws_url)
        .expect("Failed to connect Client 2");
    println!("Client 2 connected. Response status: {}", response2.status());
    
    // Set socket 1 and 2 to non-blocking read timeouts
    let underlying_stream = match socket1.get_mut() {
        tungstenite::stream::MaybeTlsStream::Plain(s) => s,
        _ => panic!("Expected plain TCP stream"),
    };
    underlying_stream.set_read_timeout(Some(Duration::from_millis(100))).expect("Failed to set read timeout");
    
    let underlying_stream2 = match socket2.get_mut() {
        tungstenite::stream::MaybeTlsStream::Plain(s) => s,
        _ => panic!("Expected plain TCP stream"),
    };
    underlying_stream2.set_read_timeout(Some(Duration::from_millis(100))).expect("Failed to set read timeout");
    
    // 5. Read telemetry from Client 1
    println!("Reading telemetry from Client 1...");
    let mut msg_received = false;
    let mut default_thresh = 0.0;
    for _ in 0..100 {
        if let Ok(Some(status)) = child_guard.0.try_wait() {
            panic!("Child process exited early during first read loop: {:?}", status);
        }
        match socket1.read() {
            Ok(msg) => {
                if let Message::Text(text) = msg {
                    println!("Client 1 received msg: {}", text);
                    if text.contains("dsp_threshold") {
                        // Parse JSON to check threshold value
                        if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                            if let Some(t) = val.get("dsp_threshold").and_then(|v| v.as_f64()) {
                                default_thresh = t;
                                msg_received = true;
                                break;
                            }
                        }
                    }
                }
            }
            Err(e) => {
                let err_str = format!("{:?}", e);
                if err_str.contains("WouldBlock") || err_str.contains("TimedOut") || err_str.contains("timed out") {
                    thread::sleep(Duration::from_millis(50));
                } else {
                    panic!("Read error on Client 1: {:?}", e);
                }
            }
        }
    }
    assert!(msg_received, "Failed to receive telemetry containing dsp_threshold from Client 1");
    println!("Default threshold is: {}", default_thresh);
    assert!((default_thresh - 5.8).abs() < 1e-5, "Expected default threshold to be 5.8, got {}", default_thresh);
    
    // 6. Send SetThreshold command via Client 2 to set threshold to 12.5
    println!("Sending SetThreshold 12.5 command via Client 2...");
    let cmd_json = r#"{"action": "SetThreshold", "value": 12.5}"#;
    socket2.send(Message::Text(cmd_json.into())).expect("Failed to send WS command");
    
    // 7. Verify that the threshold has updated by reading telemetry from Client 1
    println!("Verifying threshold update on Client 1...");
    let mut threshold_updated = false;
    for _ in 0..100 {
        if let Ok(Some(status)) = child_guard.0.try_wait() {
            panic!("Child process exited early during second read loop: {:?}", status);
        }
        match socket1.read() {
            Ok(msg) => {
                if let Message::Text(text) = msg {
                    println!("Client 1 received msg during verify: {}", text);
                    if text.contains(r#""dsp_threshold":12.5"#) || text.contains(r#""dsp_threshold": 12.5"#) {
                        threshold_updated = true;
                        break;
                    }
                }
            }
            Err(e) => {
                let err_str = format!("{:?}", e);
                if err_str.contains("WouldBlock") || err_str.contains("TimedOut") || err_str.contains("timed out") {
                    thread::sleep(Duration::from_millis(50));
                } else {
                    panic!("Read error on Client 1 during verify: {:?}", e);
                }
            }
        }
    }
    assert!(threshold_updated, "Threshold was not updated via WebSocket SetThreshold command");
    
    // 8. Check non-blocking disconnect. Client 2 disconnects.
    println!("Disconnecting Client 2...");
    drop(socket2);
    
    // Keep checking telemetry from Client 1 to ensure server keeps broadcasting
    // and did not block or crash when Client 2 disconnected.
    let mut after_disconnect_ok = false;
    for _ in 0..50 {
        if let Ok(Some(status)) = child_guard.0.try_wait() {
            panic!("Child process exited early during third read loop: {:?}", status);
        }
        match socket1.read() {
            Ok(msg) => {
                if let Message::Text(text) = msg {
                    if text.contains("dsp_threshold") {
                        after_disconnect_ok = true;
                        break;
                    }
                }
            }
            Err(e) => {
                let err_str = format!("{:?}", e);
                if err_str.contains("WouldBlock") || err_str.contains("TimedOut") || err_str.contains("timed out") {
                    thread::sleep(Duration::from_millis(50));
                } else {
                    panic!("Read error on Client 1 after disconnect: {:?}", e);
                }
            }
        }
    }
    assert!(after_disconnect_ok, "WebSocket server blocked or crashed after Client 2 disconnected");
    println!("Test websocket_server_empirical completed successfully!");
}
