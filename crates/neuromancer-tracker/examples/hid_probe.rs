//! HID probe: find MCU and OV580 output report lengths by trial writes.
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
    // OV580 command() payload: [2, cmd, subcmd, 0, 0, 0, 0]
    let pkt7 = [2u8, 0x19, 0x0, 0, 0, 0, 0];
    for n in [7usize, 8, 16, 32, 64] {
        let mut buf = vec![0u8; 1 + n];
        buf[1..1 + pkt7.len().min(n)].copy_from_slice(&pkt7[..pkt7.len().min(n)]);
        match dev.write(&buf) {
            Ok(w) => println!("  {name} write len {} (+1 ID) -> OK wrote {w}", n),
            Err(e) => println!("  {name} write len {} (+1 ID) -> FAIL {e:?}", n),
        }
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
