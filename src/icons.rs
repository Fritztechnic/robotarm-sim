//! Alle Symbole als eingebettetes SVG (weiß auf transparent, zur Laufzeit per Tint eingefärbt).

use eframe::egui::{self, load::Bytes, ImageSource};

// Jedes Icon ist ein winziges SVG im Code. Weiß auf transparent, wird zur Laufzeit eingefärbt. Keine Dateien, kein Stress
macro_rules! icons {
    ($($name:ident => $body:literal),* $(,)?) => {
        #[derive(Clone, Copy)]
        pub enum Icon { $($name),* }

        pub fn source(i: Icon) -> ImageSource<'static> {
            match i {
                $(Icon::$name => {
                    const SVG: &str = concat!(
                        r##"<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="#fff" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">"##,
                        $body, "</svg>");
                    egui::ImageSource::Bytes { uri: concat!("bytes://", stringify!($name), ".svg").into(), bytes: Bytes::Static(SVG.as_bytes()) }
                })*
            }
        }
    };
}

icons! {
    Play => r##"<path d="M7 4v16l13-8z" fill="#fff"/>"##,
    Stop => r##"<rect x="6" y="6" width="12" height="12" rx="1" fill="#fff"/>"##,
    Plus => r##"<path d="M12 5v14M5 12h14"/>"##,
    Close => r##"<path d="M6 6l12 12M18 6L6 18"/>"##,
    Up => r##"<path d="M12 19V5M6 11l6-6 6 6"/>"##,
    Goto => r##"<circle cx="12" cy="12" r="7"/><circle cx="12" cy="12" r="2" fill="#fff"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3"/>"##,
    Home => r##"<path d="M4 11l8-7 8 7M6 10v9h12v-9"/>"##,
    Repeat => r##"<path d="M17 3l3 3-3 3M4 11V9a3 3 0 013-3h13M7 21l-3-3 3-3M20 13v2a3 3 0 01-3 3H4"/>"##,
    Copy => r##"<rect x="9" y="9" width="11" height="11" rx="1"/><path d="M5 15V5a1 1 0 011-1h10"/>"##,
    Save => r##"<path d="M5 4h11l3 3v13H5zM8 4v5h7V4M8 20v-6h8v6"/>"##,
    Reset => r##"<path d="M4 12a8 8 0 102.5-5.8M4 4v5h5"/>"##,
    Grid => r##"<path d="M4 4h16v16H4zM4 9.3h16M4 14.7h16M9.3 4v16M14.7 4v16"/>"##,
    Shadow => r##"<circle cx="10" cy="9" r="4"/><ellipse cx="13" cy="18" rx="7" ry="2.5" fill="#fff" fill-opacity=".5"/>"##,
    Trail => r##"<path d="M3 19c4 0 3-12 8-12s3 10 9 10"/><circle cx="20" cy="17" r="1.6" fill="#fff"/>"##,
    Axes => r##"<path d="M12 12V3M12 12l8 5M12 12l-8 5"/>"##,
    Tag => r##"<path d="M4 4h8l8 8-8 8-8-8z"/><circle cx="8.5" cy="8.5" r="1.3" fill="#fff"/>"##,
    Marker => r##"<circle cx="12" cy="12" r="8"/><circle cx="12" cy="12" r="1.8" fill="#fff"/>"##,
    Iso => r##"<path d="M12 3l8 4.5v9L12 21l-8-4.5v-9zM4 7.5l8 4.5 8-4.5M12 12v9"/>"##,
    Front => r##"<path d="M12 3l8 4.5v9L12 21l-8-4.5v-9z"/><path d="M4 7.5l8 4.5v9l-8-4.5z" fill="#fff"/>"##,
    Side => r##"<path d="M12 3l8 4.5v9L12 21l-8-4.5v-9z"/><path d="M12 12l8-4.5v9L12 21z" fill="#fff"/>"##,
    Top => r##"<path d="M12 3l8 4.5v9L12 21l-8-4.5v-9z"/><path d="M12 3l8 4.5-8 4.5-8-4.5z" fill="#fff"/>"##,
    Sliders => r##"<path d="M4 7h10M18 7h2M4 17h2M10 17h10"/><circle cx="16" cy="7" r="2"/><circle cx="8" cy="17" r="2"/>"##,
    List => r##"<path d="M9 6h11M9 12h11M9 18h11"/><circle cx="4.5" cy="6" r="1" fill="#fff"/><circle cx="4.5" cy="12" r="1" fill="#fff"/><circle cx="4.5" cy="18" r="1" fill="#fff"/>"##,
    Sigma => r##"<path d="M18 5H7l6 7-6 7h11"/>"##,
    Eye => r##"<path d="M2 12s4-7 10-7 10 7 10 7-4 7-10 7S2 12 2 12z"/><circle cx="12" cy="12" r="3"/>"##,
}
