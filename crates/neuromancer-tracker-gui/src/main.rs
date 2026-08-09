//! `neuromancer-tracker-gui` — Linux + Windows GUI for the Nreal Light
//! head tracker.
//!
//! Embeds the minimal IMU core from `neuromancer-tracker` (lib): the IMU
//! runs on a background thread (`ImuTracker::next_pose` → Mahony), and the
//! egui UI shows:
//!
//! - a **Settings menu** whose items mirror the CLI switches
//!   (`--no-udp`, `--host`, `--port`, `--udp-rate`, `--hud-rate`,
//!   `--gyro-calib`, `--kp`, `--ki`, `--units`), persisted to the same
//!   TOML settings file the CLI reads/writes (`Settings::config_path`);
//! - a **HUD** (YPR readout + IMU rate);
//! - an optional **3D wireframe cube** visualizing the glasses rotation
//!   (toggle on/off), drawn with painter-side 3D→2D projection (no GPU
//!   dependency beyond what egui already uses).
//!
//! The rotation quaternion comes from `neuromancer_ahrs::Quat` (world→body,
//! same convention as the tracker).

// GUI app on Windows: stay a console-subsystem app (so the GUI window shows
// reliably) but hide the console window after startup via ShowWindow(SW_HIDE).
// Using windows_subsystem=windows made the GUI window invisible on the Mini
// (eframe keeps it hidden until first paint; the subsystem switch broke
// that), so we avoid it.

#[cfg(target_os = "windows")]
fn hide_console() {
    // ShowWindow on the console, not the GUI window.
    unsafe {
        let console = windows_sys::Win32::System::Console::GetConsoleWindow();
        if !console.is_null() {
            let _ = windows_sys::Win32::UI::WindowsAndMessaging::ShowWindow(console, 0); // SW_HIDE
        }
    }
}

use std::sync::mpsc;
use std::sync::{Arc, Mutex};

use eframe::egui;
use neuromancer_ahrs::Quat;
use neuromancer_tracker::{ImuTracker, Pose, Settings, UdpSink};

/// Debug startup log (works even with windows_subsystem, where stdout is
/// invisible). Append-only; harmless in production.
fn dbg_log(msg: &str) {
    use std::io::Write;
    if let Ok(mut f) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open("gui_debug.log")
    {
        let _ = writeln!(f, "{}", msg);
    }
}

// ---------------------------------------------------------------------------
// Shared state between the IMU thread and the UI
// ---------------------------------------------------------------------------

#[derive(Default)]
struct LiveState {
    pose: Option<Pose>,
    rate_hz: f64,
    calibrating: bool,
    error: Option<String>,
}

enum ImuMsg {
    Pose(Pose),
    Rate(f64),
    Calibrating(bool),
    Error(String),
}

/// Commands from the UI to the tracker thread (applied live).
#[derive(Debug, Clone, Copy)]
enum JitterCtrl {
    /// Enable/disable + alpha (0..1). Applied on the next loop iteration.
    SetJitter { enabled: bool, alpha: f64 },
}

// ---------------------------------------------------------------------------
// App
// ---------------------------------------------------------------------------

struct TrackerApp {
    settings: Settings,
    live: Arc<Mutex<LiveState>>,
    tx: mpsc::Sender<ImuMsg>,
    rx: mpsc::Receiver<ImuMsg>,
    udp: Option<UdpSink>,
    running: bool,
    last_settings_note: Option<String>,
    /// Live control of the tracker thread (jitter filter toggle).
    jitter_tx: mpsc::Sender<JitterCtrl>,
    /// Orientation reference (the "zero"). `None` = identity (no reset yet).
    /// Once set, every displayed/emitted pose is relative to it until a new
    /// reset replaces it — the zero persists until a new zero is chosen.
    zero_ref: Option<Quat>,
}

impl TrackerApp {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let settings = Settings::load();
        let (tx, rx) = mpsc::channel::<ImuMsg>();
        let (jitter_tx, jitter_rx) = mpsc::channel::<JitterCtrl>();

        // UDP sink (created once; rebuilt when settings change).
        let udp = if settings.no_udp {
            None
        } else {
            UdpSink::bind(&settings.host, settings.port, settings.udp_rate).ok()
        };

        let mut app = TrackerApp {
            settings,
            live: Arc::new(Mutex::new(LiveState::default())),
            tx,
            rx,
            udp,
            running: false,
            last_settings_note: None,
            jitter_tx,
            zero_ref: None,
        };
        let _ = cc; // theme/fonts later if needed
        app.start_tracker(jitter_rx);
        app
    }

    /// Current pose transformed by the reset reference (the displayed pose).
    /// Returns `None` if no pose yet. Does NOT lock `self.live` — the caller
    /// must pass the pose (avoids re-entrant locking the panel already holds).
    fn displayed(&self, pose: &Pose) -> (Quat, [f64; 3]) {
        let q = match self.zero_ref {
            Some(z) => z.conjugate() * pose.q, // relative to the zero
            None => pose.q,
        };
        (q, neuromancer_ahrs::quat_to_ypr(q))
    }

    /// Re-zero: the current orientation becomes 0,0,0 until the next reset.
    fn reset_zero(&mut self) {
        if let Some(pose) = self.live.lock().unwrap().pose {
            self.zero_ref = Some(pose.q);
            self.last_settings_note = Some("zeroed — current orientation is now 0,0,0 (Space to re-zero)".to_string());
        }
    }

    fn start_tracker(&mut self, jitter_rx: mpsc::Receiver<JitterCtrl>) {
        let settings = self.settings.clone();
        let tx = self.tx.clone();
        std::thread::spawn(move || {
            let mut tracker = match ImuTracker::open() {
                Ok(t) => {
                    dbg_log("imu: open OK");
                    t
                }
                Err(e) => {
                    dbg_log(&format!("imu: open FAILED: {e}"));
                    tx.send(ImuMsg::Error(e)).ok();
                    return;
                }
            };
            tracker.configure(&settings);
            dbg_log("imu: configure OK");
            let calibrating = settings.gyro_calib > 0.0;
            tx.send(ImuMsg::Calibrating(calibrating)).ok();
            let mut n_poses = 0u32;
            loop {
                // Apply live UI commands (jitter toggle) before each sample.
                while let Ok(cmd) = jitter_rx.try_recv() {
                    match cmd {
                        JitterCtrl::SetJitter { enabled, alpha } => {
                            tracker.set_jitter_filter(enabled, alpha);
                            dbg_log(&format!("imu: jitter filter -> enabled={enabled} alpha={alpha}"));
                        }
                    }
                }
                match tracker.next_pose() {
                    Ok(None) => {
                        if calibrating && !tracker.calibrating() {
                            tx.send(ImuMsg::Calibrating(false)).ok();
                        }
                        continue;
                    }
                    Ok(Some(pose)) => {
                        n_poses += 1;
                        if n_poses == 1 {
                            dbg_log(&format!("imu: first pose {pose:?}"));
                        } else if n_poses.is_multiple_of(500) {
                            dbg_log(&format!("imu: {n_poses} poses, rate {:.0} Hz", tracker.last_rate_hz));
                        }
                        tx.send(ImuMsg::Pose(pose)).ok();
                        tx.send(ImuMsg::Rate(tracker.last_rate_hz)).ok();
                    }
                    Err(e) => {
                        dbg_log(&format!("imu: read error: {e}"));
                        tx.send(ImuMsg::Error(e)).ok();
                        break;
                    }
                }
            }
        });
        self.running = true;
    }
}

impl eframe::App for TrackerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        static FIRST: std::sync::Once = std::sync::Once::new();
        FIRST.call_once(|| dbg_log("update: first frame"));
        // Log every ~2 s that the UI loop is alive (wall-clock based, not
        // frame-count — the repaint loop can run at >1 kHz).
        {
            use std::sync::atomic::{AtomicU64, Ordering};
            static LAST_LOG: AtomicU64 = AtomicU64::new(0);
            let now_ms = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_millis() as u64)
                .unwrap_or(0);
            let last = LAST_LOG.load(Ordering::Relaxed);
            if now_ms.saturating_sub(last) >= 2000 {
                LAST_LOG.store(now_ms, Ordering::Relaxed);
                dbg_log("update: alive");
            }
        }        // The IMU thread streams poses continuously; egui only repaints when
        // told to, so request a repaint every frame (and again shortly
        // after) — otherwise the HUD/cube freeze between input events,
        // appearing to "run ~0.5 s then hang ~1 s" in bursts.
        ctx.request_repaint();
        ctx.request_repaint_after(std::time::Duration::from_millis(16)); // ~60 fps

        // --- Space key: re-zero (when the window is focused) ---------------
        if ctx.input(|i| i.key_pressed(egui::Key::Space)) {
            self.reset_zero();
        }

        // Drain the IMU thread's messages.
        while let Ok(msg) = self.rx.try_recv() {
            match msg {
                ImuMsg::Pose(p) => {
                    // Send to Opentrack UDP (rate-gated inside the sink),
                    // relative to the current zero.
                    let (_, ypr) = self.displayed(&p);
                    if let Some(s) = self.udp.as_mut() {
                        s.send(if self.settings.rad { ypr } else { ypr.map(f64::to_degrees) });
                    }
                    self.live.lock().unwrap().pose = Some(p);
                }
                ImuMsg::Rate(r) => self.live.lock().unwrap().rate_hz = r,
                ImuMsg::Calibrating(c) => self.live.lock().unwrap().calibrating = c,
                ImuMsg::Error(e) => self.live.lock().unwrap().error = Some(e),
            }
        }

        egui::TopBottomPanel::top("menu").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("Nreal Light Tracker");
                ui.separator();
                if ui.button("Start").clicked() && !self.running {
                    // Fresh control channel: the receiver is moved into the
                    // new tracker thread, the sender replaces this one.
                    let (jitter_tx, jitter_rx) = mpsc::channel::<JitterCtrl>();
                    self.jitter_tx = jitter_tx;
                    self.start_tracker(jitter_rx);
                }
                if ui.button("Stop").clicked() {
                    self.running = false;
                    // Stopping the USB thread isn't instantaneous; a full stop
                    // would need a cancel token. For now Stop just detaches
                    // from the UI state (the thread exits on USB error).
                }
                ui.separator();
                if ui.button("Reset 0,0,0 (Space)").clicked() {
                    self.reset_zero();
                }
                ui.separator();
                settings_menu(ui, &mut self.settings, &mut self.last_settings_note, &self.jitter_tx);
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            let live = self.live.lock().unwrap();

            // Error banner — bold, readable.
            if let Some(e) = &live.error {
                ui.colored_label(
                    egui::Color32::from_rgb(230, 60, 60),
                    egui::RichText::new(format!("IMU error: {e}")).strong(),
                );
            }
            if live.calibrating {
                // Darker yellow + bold for contrast on light backgrounds.
                ui.colored_label(
                    egui::Color32::from_rgb(190, 140, 0),
                    egui::RichText::new("gyro bias calibration: keep the device still ...").strong().size(14.0),
                );
            }
            if self.zero_ref.is_some() {
                ui.colored_label(
                    egui::Color32::from_rgb(0, 160, 120),
                    egui::RichText::new("zeroed — angles relative to reset (Space to re-zero)").strong(),
                );
            }

            // HUD — displayed pose is relative to the zero. Copy the pose out
            // of the lock first; displayed() does not lock again (the guard
            // is still held here, and Mutex is not reentrant).
            let display = live.pose.as_ref().map(|p| self.displayed(p));
            if let Some((_, ypr)) = display {
                let (yaw, pitch, roll) = if self.settings.rad {
                    (ypr[0], ypr[1], ypr[2])
                } else {
                    (ypr[0].to_degrees(), ypr[1].to_degrees(), ypr[2].to_degrees())
                };
                let unit = if self.settings.rad { "rad" } else { "°" };
                ui.label(format!("YAW {yaw:8.3} {unit}   PITCH {pitch:8.3} {unit}   ROLL {roll:8.3} {unit}"));
            } else {
                ui.label("no pose yet");
            }
            ui.label(format!("IMU rate: {:.0} Hz", live.rate_hz));
            drop(live);

            if let Some(note) = &self.last_settings_note {
                ui.label(note.clone());
            }

            // 3D cube visualization (toggle) — rotated by the zeroed pose.
            if self.settings.show_cube {
                ui.separator();
                ui.label("Glasses rotation (cube):");
                cube_panel(ui, display.map(|(q, _)| q));
            }
        });
    }
}

// ---------------------------------------------------------------------------
// Settings menu — one widget per CLI switch.
// ---------------------------------------------------------------------------

fn settings_menu(
    ui: &mut egui::Ui,
    settings: &mut Settings,
    note: &mut Option<String>,
    jitter_tx: &mpsc::Sender<JitterCtrl>,
) {
    let mut save = false;
    let mut apply_jitter = false;
    ui.menu_button("Settings", |ui| {
        ui.checkbox(&mut settings.no_udp, "Disable UDP (--no-udp)");
        ui.add(egui::DragValue::new(&mut settings.udp_rate).prefix("UDP rate (--udp-rate): ").range(1.0..=200.0));
        ui.add(egui::DragValue::new(&mut settings.hud_rate).prefix("HUD rate (--hud-rate): ").range(0.0..=30.0));
        ui.add(egui::DragValue::new(&mut settings.gyro_calib).prefix("Gyro calib (--gyro-calib): ").range(0.0..=10.0));
        ui.add(egui::DragValue::new(&mut settings.kp).prefix("Mahony kp (--kp): ").range(0.0..=5.0));
        ui.add(egui::DragValue::new(&mut settings.ki).prefix("Mahony ki (--ki): ").range(0.0..=0.1));
        ui.horizontal(|ui| {
            ui.label("Units (--units):");
            ui.radio_value(&mut settings.rad, false, "deg");
            ui.radio_value(&mut settings.rad, true, "rad");
        });
        ui.horizontal(|ui| {
            ui.label("UDP host (--host):");
            ui.text_edit_singleline(&mut settings.host);
        });
        ui.add(egui::DragValue::new(&mut settings.port).prefix("UDP port (--port): ").range(1..=65535));
        ui.checkbox(&mut settings.show_cube, "Show 3D cube visualization");
        ui.separator();
        // Jitter filter — applied LIVE via the control channel (no restart).
        if ui.checkbox(&mut settings.jitter_filter, "Jitter filter (--jitter-filter)").changed() {
            apply_jitter = true;
        }
        if ui
            .add(egui::DragValue::new(&mut settings.jitter_alpha).prefix("Jitter alpha (--jitter-alpha): ").range(0.0..=1.0).speed(0.005))
            .changed()
        {
            apply_jitter = true;
        }
        if settings.jitter_filter {
            ui.label("smooths the constant headset jitter (smaller = smoother)");
        }
        ui.separator();
        if ui.button("Save settings (TOML)").clicked() {
            save = true;
        }
    });
    if apply_jitter {
        let _ = jitter_tx.send(JitterCtrl::SetJitter {
            enabled: settings.jitter_filter,
            alpha: settings.jitter_alpha,
        });
    }
    if save {
        match settings.save() {
            Ok(path) => {
                *note = Some(format!("settings saved to {}", path.display()));
            }
            Err(e) => *note = Some(format!("settings save failed: {e}")),
        }
    }
}

// ---------------------------------------------------------------------------
// 3D wireframe cube — painter-side projection, no GPU code of our own.
// ---------------------------------------------------------------------------

/// Draw a wireframe cube rotated by `q` (world→body quaternion), axes
/// colored, plus a ground plane hint. Uses egui's Painter with a simple
/// orthographic projection of the 8 cube corners.
fn cube_panel(ui: &mut egui::Ui, q: Option<neuromancer_ahrs::Quat>) {
    let desired = egui::vec2(260.0, 260.0);
    let (rect, _) = ui.allocate_exact_size(desired, egui::Sense::hover());
    let painter = ui.painter_at(rect);
    let center = rect.center();

    // Cube corners in body coordinates (unit cube, half-extent 0.6).
    let corners = [
        [-1.0, -1.0, -1.0],
        [1.0, -1.0, -1.0],
        [1.0, 1.0, -1.0],
        [-1.0, 1.0, -1.0],
        [-1.0, -1.0, 1.0],
        [1.0, -1.0, 1.0],
        [1.0, 1.0, 1.0],
        [-1.0, 1.0, 1.0],
    ];
    // Cube edges (index pairs).
    let edges = [
        (0, 1), (1, 2), (2, 3), (3, 0), // back face
        (4, 5), (5, 6), (6, 7), (7, 4), // front face
        (0, 4), (1, 5), (2, 6), (3, 7), // connectors
    ];

    let q = q.unwrap_or(neuromancer_ahrs::Quat::new(1.0, 0.0, 0.0, 0.0));

    // Project: rotate each corner by q, then a fixed view rotation so we
    // see the cube from a 3/4 angle, then orthographic (x→right, y→down).
    let view = view_rotation();
    let projected: Vec<egui::Pos2> = corners
        .iter()
        .map(|c| {
            let v = q.rotate_vector(*c);
            let v = view_rotate(view, v);
            let s = 55.0; // scale (px per unit)
            egui::pos2(center.x + (v[0] * s) as f32, center.y - (v[1] * s) as f32)
        })
        .collect();

    // Depth cue: edges on the far side get a dimmer color. Simple heuristic:
    // use the average z of the two endpoints.
    for (a, b) in edges {
        let za = corners[a][2];
        let zb = corners[b][2];
        let depth = (za + zb) / 2.0;
        let color = if depth < 0.0 {
            egui::Color32::from_rgb(120, 120, 140)
        } else {
            egui::Color32::from_rgb(200, 200, 220)
        };
        painter.line_segment([projected[a], projected[b]], egui::Stroke::new(1.5_f32, color));
    }

    // Axis arrows (body axes after rotation): X red, Y green, Z blue.
    let axes: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let colors = [
        egui::Color32::from_rgb(230, 80, 80),
        egui::Color32::from_rgb(80, 220, 80),
        egui::Color32::from_rgb(80, 130, 240),
    ];
    for (axis, color) in axes.iter().zip(colors.iter()) {
        let v = q.rotate_vector(*axis);
        let v = view_rotate(view, v);
        let end = egui::pos2(
            center.x + (v[0] * 75.0) as f32,
            center.y - (v[1] * 75.0) as f32,
        );
        painter.line_segment([center, end], egui::Stroke::new(2.0_f32, *color));
    }

    // Caption.
    painter.text(
        rect.left_bottom() + egui::vec2(2.0, 12.0),
        egui::Align2::LEFT_BOTTOM,
        format!("q = ({:.3}, {:.3}, {:.3}, {:.3})", q.w, q.x, q.y, q.z),
        egui::FontId::monospace(10.0),
        egui::Color32::GRAY,
    );
}

/// Fixed 3/4 view rotation: rotate the cube so we see three faces.
fn view_rotation() -> [[f64; 3]; 3] {
    // Rotation matrix: R = Rx(0.5)·Ry(-0.6) (about x then y).
    let (a, b) = (0.5f64, -0.6f64);
    let (ca, sa) = (a.cos(), a.sin());
    let (cb, sb) = (b.cos(), b.sin());
    [
        [cb, 0.0, sb],
        [sa * sb, ca, -sa * cb],
        [-ca * sb, sa, ca * cb],
    ]
}

fn view_rotate(r: [[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    [
        r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
        r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
        r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2],
    ]
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

fn main() -> eframe::Result<()> {
    #[cfg(target_os = "windows")]
    hide_console();
    dbg_log("main: start");
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Nreal Light Tracker")
            .with_inner_size([480.0, 440.0]),
        ..Default::default()
    };
    dbg_log("main: run_native");
    let r = eframe::run_native(
        "neuromancer-tracker-gui",
        options,
        Box::new(|cc| {
            dbg_log("creation ctx: building app");
            Ok(Box::new(TrackerApp::new(cc)))
        }),
    );
    dbg_log("main: run_native returned");
    r
}
