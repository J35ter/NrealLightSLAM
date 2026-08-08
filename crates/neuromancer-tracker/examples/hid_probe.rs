//! HID probe: run the real NrealLight::new() + read a few IMU events.
//! Usage: cargo run --release -p neuromancer-tracker --example hid_probe
use ar_drivers::ARGlasses;

fn main() {
    println!("=== ar_drivers::NrealLight::new() ===");
    match ar_drivers::nreal_light::NrealLight::new() {
        Ok(mut g) => {
            println!("NrealLight::new() OK — reading events (8 s)...");
            let mut got = 0;
            for _ in 0..40 {
                match g.read_event() {
                    Ok(ev) => {
                        got += 1;
                        let brief = format!("{ev:?}");
                        let brief = brief.chars().take(110).collect::<String>();
                        println!("  event #{got}: {brief}");
                        if got >= 4 {
                            break;
                        }
                    }
                    Err(e) => {
                        println!("  read_event err: {e:?}");
                        break;
                    }
                }
            }
            println!("events received: {got}");
        }
        Err(e) => eprintln!("NrealLight::new() FAILED: {e:?}"),
    }
}
