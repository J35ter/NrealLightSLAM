//! HID probe: MCU read behavior — write @3 then read; also enumerate paths.
//! Usage: cargo run --release -p neuromancer-tracker --example hid_probe
use hidapi::HidApi;

fn main() {
    let api = HidApi::new().unwrap();
    // List all 0486:573c devices with paths
    println!("=== MCU devices ===");
    for d in api.device_list().filter(|d| d.vendor_id() == 0x0486 && d.product_id() == 0x573c) {
        println!("  path={:?}", d.path());
    }
    // open the FIRST by enumeration (what open(vid,pid) does)
    let mcu = api.open(0x0486, 0x573c).expect("MCU open");
    println!("MCU open OK — write @3 then read");
    let mut pkt = [0u8; 64];
    pkt[0] = 2; pkt[1] = b':'; pkt[2] = b'@'; pkt[3] = b':'; pkt[4] = b'3'; pkt[5] = b':';
    pkt[6] = b'1';
    let rest = b":0:00000000:3";
    pkt[7..7 + rest.len()].copy_from_slice(rest);
    println!("  write: {:?}", mcu.write(&pkt));
    for i in 0..6 {
        let mut buf = [0u8; 64];
        match mcu.read_timeout(&mut buf, 500) {
            Ok(0) => println!("  read#{i}: timeout (0)"),
            Ok(n) => println!("  read#{i}: {n} bytes [{:02x?}]", &buf[..n.min(12)]),
            Err(e) => {
                println!("  read#{i} ERR: {e:?}");
                break;
            }
        }
    }
}
