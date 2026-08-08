//! HID probe: replicate read_config() fully (loop 0x15,0x0 until buf[1]!=1)
//! Usage: cargo run --release -p neuromancer-tracker --example hid_probe
use hidapi::HidApi;

fn cmd_ack(dev: &hidapi::HidDevice, cmd: u8, sub: u8) -> Option<Vec<u8>> {
    dev.write(&[2, cmd, sub, 0, 0, 0, 0]).ok()?;
    for _ in 0..64 {
        let mut buf = [0u8; 0x80];
        match dev.read_timeout(&mut buf, 250) {
            Ok(0) => return None,
            Ok(n) => {
                if buf[0] == 2 {
                    return Some(buf[..n].to_vec());
                }
            }
            Err(_) => return None,
        }
    }
    None
}

fn main() {
    let api = HidApi::new().unwrap();
    let ov = api.open(0x05a9, 0x0680).expect("OV580 open");
    println!("OV580 open OK");
    println!("cmd 0x19,0x0 -> {:?}", cmd_ack(&ov, 0x19, 0x0).map(|v| v[..8].to_vec()));
    println!("cmd 0x14,0x0 -> {:?}", cmd_ack(&ov, 0x14, 0x0).map(|v| v[..8].to_vec()));
    let mut total = 0usize;
    let mut chunks = 0usize;
    loop {
        match cmd_ack(&ov, 0x15, 0x0) {
            Some(part) if part.len() > 1 && part[1] == 1 => {
                let len = part[2] as usize;
                chunks += 1;
                total += len;
                if chunks <= 3 {
                    println!("chunk#{chunks}: buf[0..3]=[{:02x?}] len={len}", &part[..3]);
                }
            }
            other => {
                println!("config loop ended after {chunks} chunks, {total} bytes: {:?}", other.map(|v| v[..4].to_vec()));
                break;
            }
        }
    }
    println!("done");
}
