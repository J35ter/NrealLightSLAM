//! HID probe: step the full OV580 init: 0x19,0x0 -> 0x14,0x0 -> 0x15,0x0 -> 0x19,0x1
//! Usage: cargo run --release -p neuromancer-tracker --example hid_probe
use hidapi::HidApi;

fn cmd(dev: &hidapi::HidDevice, cmd: u8, sub: u8, label: &str) -> bool {
    println!("--- {label}: command({cmd:#x},{sub:#x}) ---");
    if let Err(e) = dev.write(&[2, cmd, sub, 0, 0, 0, 0]) {
        eprintln!("  write FAIL {e:?}");
        return false;
    }
    // read until ack (buf[0]==2), max 16 reads of 250ms
    for i in 0..16 {
        let mut buf = [0u8; 0x80];
        match dev.read_timeout(&mut buf, 250) {
            Ok(0) => {}
            Ok(n) => {
                if buf[0] == 2 {
                    println!("  ack#{i}: [{:02x?}...]", &buf[..n.min(8)]);
                    return true;
                }
                // else: skip non-ack (e.g. IMU 01 reports)
            }
            Err(e) => {
                eprintln!("  read#{i} ERR {e:?}");
                return false;
            }
        }
    }
    eprintln!("  no ack in 16 reads (PacketTimeout equivalent)");
    false
}

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
    let _ = cmd(&ov, 0x19, 0x0, "turn IMU stream off");
    let _ = cmd(&ov, 0x14, 0x0, "config start");
    let _ = cmd(&ov, 0x15, 0x0, "config chunk");
    let _ = cmd(&ov, 0x19, 0x1, "turn IMU stream on");
}
