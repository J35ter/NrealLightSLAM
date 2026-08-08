//! HID probe: step through the OV580 init commands manually to find where
//! PacketTimeout occurs on Windows.
//! Usage: cargo run --release -p neuromancer-tracker --example hid_probe
use hidapi::HidApi;

fn main() {
    let api = match HidApi::new() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("HidApi::new() failed: {e:?}");
            return;
        }
    };
    let ov = match api.open(0x05a9, 0x0680) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("OV580 open FAILED: {e:?}");
            return;
        }
    };
    println!("OV580 open OK");

    // Step 1: command(0x19, 0x0) — turn off IMU stream (from Ov580::new).
    println!("--- command(0x19, 0x0) write ---");
    match ov.write(&[2, 0x19, 0x0, 0, 0, 0, 0]) {
        Ok(w) => println!("  write OK ({w} bytes)"),
        Err(e) => {
            eprintln!("  write FAIL: {e:?}");
            return;
        }
    }
    // Then read the ack (command() reads up to 64 x 250ms).
    println!("--- read ack ---");
    let mut acked = false;
    for i in 0..8 {
        let mut buf = [0u8; 0x80];
        match ov.read_timeout(&mut buf, 250) {
            Ok(0) => println!("  read {i}: timeout (0 bytes)"),
            Ok(n) => {
                println!("  read {i}: {n} bytes: {:02x?}...", &buf[..n.min(16)]);
                if buf[0] == 2 {
                    acked = true;
                    println!("  ACKED (buf[0]==2)");
                    break;
                }
            }
            Err(e) => {
                eprintln!("  read {i} ERR: {e:?}");
                break;
            }
        }
    }
    println!("acked: {acked}");
}
