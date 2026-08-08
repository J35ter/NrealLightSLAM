//! HID probe: find the Nreal MCU's output report length by trial writes.
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
    let mcu = match api.open(0x0486, 0x573c) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("MCU open FAILED: {e:?}");
            return;
        }
    };
    println!("MCU open OK");

    // The "SDK working" command packet from nreal_light.rs run_command:
    // serialize(Packet { category: b'@', cmd_id: b'3', data: vec![b'1'] }).
    let mut pkt = [0u8; 64];
    pkt[0] = 2; pkt[1] = b':'; pkt[2] = b'@'; pkt[3] = b':'; pkt[4] = b'3'; pkt[5] = b':';
    pkt[6] = b'1';
    pkt[7..9].copy_from_slice(b":0");
    pkt[9] = b':';
    // crc placeholder — any bytes fine for the write test
    for (i, b) in "00000000:3".bytes().enumerate() {
        pkt[10 + i] = b;
    }

    // Try write lengths: report-ID + N bytes, N in {64, 32, 16, 8}.
    for n in [64usize, 32, 16, 8] {
        let mut buf = vec![0u8; 1 + n];
        buf[1..1 + n].copy_from_slice(&pkt[..n]);
        match mcu.write(&buf) {
            Ok(w) => println!("write len {} (+1 ID) -> OK, wrote {w}", n),
            Err(e) => println!("write len {} (+1 ID) -> FAIL {e:?}", n),
        }
    }
    // Also try without the report ID.
    match mcu.write(&pkt) {
        Ok(w) => println!("write len 64 (no ID) -> OK, wrote {w}"),
        Err(e) => println!("write len 64 (no ID) -> FAIL {e:?}"),
    }
}
