//! HID probe replicating NrealLight::new(): open MCU + OV580, then init cmds.
//! Usage: cargo run --release -p neuromancer-tracker --example hid_probe
use ar_drivers::ARGlasses;
use hidapi::HidApi;

fn main() {
    let api = match HidApi::new() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("HidApi::new() failed: {e:?}");
            return;
        }
    };
    println!("=== open MCU 0486:573c ===");
    let mcu = match api.open(0x0486, 0x573c) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("MCU open FAILED: {e:?}");
            return;
        }
    };
    println!("MCU open OK");
    println!("=== open OV580 05a9:0680 ===");
    let ov580 = match api.open(0x05a9, 0x0680) {
        Ok(d) => d,
        Err(e) => {
            eprintln!("OV580 open FAILED: {e:?}");
            return;
        }
    };
    println!("OV580 open OK");

    // Now the full NrealLight::new() path (via the crate), which also sends
    // init commands. If this fails we get the real error with detail.
    println!("=== ar_drivers::NrealLight::new() ===");
    match ar_drivers::nreal_light::NrealLight::new() {
        Ok(mut g) => {
            println!("NrealLight::new() OK — reading an event (3 s timeout)...");
            let mut got = 0;
            for _ in 0..30 {
                match g.read_event() {
                    Ok(ev) => {
                        got += 1;
                        println!("  event #{got}: {ev:?}");
                        if got >= 3 {
                            break;
                        }
                    }
                    Err(e) => {
                        println!("  read_event err: {e:?}");
                        break;
                    }
                }
            }
            println!("events: {got}");
        }
        Err(e) => eprintln!("NrealLight::new() FAILED: {e:?}"),
    }
    let _ = (mcu, ov580);
}
