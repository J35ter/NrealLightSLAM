//! HID probe: try write(), send_output_report(), send_feature_report() on
//! MCU + OV580 to find the working OV580 command path on Windows.
//! Usage: cargo run --release -p neuromancer-tracker --example hid_probe
use hidapi::HidApi;

fn trial(api: &HidApi, vid: u16, pid: u16, name: &str) {
    let dev = match api.open(vid, pid) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("{name} open FAILED: {e:?}");
            return;
        }
    };
    println!("{name} open OK");
    let pkt = [2u8, 0x19, 0x0, 0, 0, 0, 0]; // OV580 command(0x19,0): turn IMU stream off
    // write() — interrupt OUT endpoint
    match dev.write(&pkt) {
        Ok(w) => println!("  {name} write() -> OK wrote {w}"),
        Err(e) => println!("  {name} write() -> FAIL {e:?}"),
    }
    // send_output_report() — control endpoint Set_Report(Output)
    let mut buf = vec![0u8; 65];
    buf[1..8].copy_from_slice(&pkt);
    match dev.send_output_report(&buf) {
        Ok(_) => println!("  {name} send_output_report() -> OK"),
        Err(e) => println!("  {name} send_output_report() -> FAIL {e:?}"),
    }
    // send_feature_report() — control endpoint Set_Report(Feature)
    let mut fbuf = vec![0u8; 65];
    fbuf[1..8].copy_from_slice(&pkt);
    match dev.send_feature_report(&fbuf) {
        Ok(_) => println!("  {name} send_feature_report() -> OK"),
        Err(e) => println!("  {name} send_feature_report() -> FAIL {e:?}"),
    }
}

fn main() {
    let api = match HidApi::new() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("HidApi::new() failed: {e:?}");
            return;
        }
    };
    trial(&api, 0x0486, 0x573c, "MCU");
    trial(&api, 0x05a9, 0x0680, "OV580");
}
