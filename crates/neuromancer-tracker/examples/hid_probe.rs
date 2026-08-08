//! HID probe: time MCU write vs OV580 read to find the stall source.
//! Usage: cargo run --release -p neuromancer-tracker --example hid_probe
use std::time::Instant;
use hidapi::HidApi;

fn main() {
    let api = HidApi::new().unwrap();
    let mcu = api.open(0x0486, 0x573c).unwrap();
    let ov = api.open(0x05a9, 0x0680).unwrap();

    // Turn the IMU stream on first.
    ov.write(&[2, 0x19, 0x1, 0, 0, 0, 0]).unwrap();
    let mut hb = [0u8; 64];
    hb[0] = 2; hb[1] = b':'; hb[2] = b'@'; hb[3] = b':'; hb[4] = b'K'; hb[5] = b':';
    let rest = b":0:00000000:3";
    hb[6..6 + rest.len()].copy_from_slice(rest);

    println!("t(s)\tMCU_write(ms)\tOV_read(ms)\tOV_bytes");
    let t0 = Instant::now();
    let mut next_hb = Instant::now();
    for i in 0..40 {
        let t = t0.elapsed().as_secs_f64();
        // heartbeat every 250ms like send_heartbeat_if_needed
        let mut hb_ms = 0.0f64;
        if Instant::now() >= next_hb {
            next_hb = Instant::now() + std::time::Duration::from_millis(250);
            let w0 = Instant::now();
            let _ = mcu.write(&hb);
            hb_ms = w0.elapsed().as_secs_f64() * 1000.0;
        }
        // read one OV580 report (like ov580.read_packet)
        let mut buf = [0u8; 0x80];
        let r0 = Instant::now();
        let n = ov.read_timeout(&mut buf, 250).unwrap_or(0);
        let ov_ms = r0.elapsed().as_secs_f64() * 1000.0;
        println!("{t:.3}\t{hb_ms:.1}\t\t{ov_ms:.1}\t\t{n}");
        std::thread::sleep(std::time::Duration::from_millis(5));
    }
}
