//! Radio phraseology: transmissions are assembled from a pre-rendered
//! vocabulary (`assets/audio/atc/vocab.tsv`, rendered per voice by
//! `assets/audio/tools/gen_atc.py`) — words and whole stock phrases — with
//! the spoken form and the subtitle built side by side ([`Utt`]).
//!
//! Numbers follow ICAO / UK CAP 413 practice: digits one by one ("one zero
//! one three", "niner"), whole thousands and hundreds for altitudes ("three
//! thousand five hundred feet"), flight levels in digits except whole
//! hundreds ("flight level one hundred"), frequencies with "decimal" and
//! trailing zeros dropped ("one one eight decimal five"), runways with
//! left / right / centre.

use std::collections::HashMap;
use std::sync::OnceLock;

const VOCAB_TSV: &str = include_str!("../../assets/audio/atc/vocab.tsv");

/// The phrase vocabulary: key (lower-case spoken text) → index.
pub struct Vocab {
    pub keys: Vec<&'static str>,
    /// Which voices speak it (P pilot, C controllers, A ATIS).
    pub tags: Vec<&'static str>,
    map: HashMap<&'static str, u16>,
    /// Longest key in words (greedy matching window).
    max_words: usize,
}

impl Vocab {
    pub fn get(&self, key: &str) -> Option<u16> {
        self.map.get(key).copied()
    }
    pub fn len(&self) -> usize {
        self.keys.len()
    }
}

pub fn vocab() -> &'static Vocab {
    static V: OnceLock<Vocab> = OnceLock::new();
    V.get_or_init(|| {
        let mut keys = Vec::new();
        let mut tags = Vec::new();
        for l in VOCAB_TSV.lines().filter(|l| !l.starts_with('#') && !l.trim().is_empty()) {
            let mut f = l.split('\t');
            let k = f.next().unwrap_or("");
            let _tts = f.next();
            keys.push(k);
            tags.push(f.next().unwrap_or("PCA"));
        }
        let map = keys.iter().enumerate().map(|(i, k)| (*k, i as u16)).collect();
        let max_words = keys.iter().map(|k| k.split(' ').count()).max().unwrap_or(1);
        Vocab { keys, tags, map, max_words }
    })
}

/// One spoken element.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Tok {
    /// A vocabulary entry.
    Word(u16),
    /// Silence (s): commas, full stops, between number groups.
    Gap(f32),
}

/// Pause lengths (s).
pub const GAP_COMMA: f32 = 0.16;
pub const GAP_STOP: f32 = 0.34;

/// A transmission being built: what is said and what the subtitle shows.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Utt {
    pub toks: Vec<Tok>,
    pub text: String,
    /// Spoken words not in the vocabulary (tests assert there are none).
    pub missing: Vec<String>,
}

pub const DIGITS: [&str; 10] = ["zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "niner"];
pub const LETTERS: [&str; 26] = [
    "alpha", "bravo", "charlie", "delta", "echo", "foxtrot", "golf", "hotel", "india", "juliett", "kilo", "lima", "mike", "november", "oscar", "papa", "quebec", "romeo", "sierra",
    "tango", "uniform", "victor", "whiskey", "x-ray", "yankee", "zulu",
];

pub fn digit_word(c: char) -> Option<&'static str> {
    c.to_digit(10).map(|d| DIGITS[d as usize])
}
pub fn letter_word(c: char) -> Option<&'static str> {
    c.is_ascii_alphabetic().then(|| LETTERS[(c.to_ascii_uppercase() as u8 - b'A') as usize])
}

/// Spell letters and digits ("A10" → "alpha one zero"); other characters are dropped.
pub fn spell(s: &str) -> String {
    s.chars().filter_map(|c| digit_word(c).or_else(|| letter_word(c))).collect::<Vec<_>>().join(" ")
}

/// Digits one by one ("1013" → "one zero one three").
pub fn digits(n: u32) -> String {
    spell(&n.to_string())
}

/// "27R" → "two seven right"; "09" → "zero niner"; "36C" → "three six centre".
pub fn runway_words(id: &str) -> String {
    let mut w: Vec<&str> = Vec::new();
    for c in id.chars() {
        match c.to_ascii_uppercase() {
            'L' => w.push("left"),
            'R' => w.push("right"),
            'C' => w.push("centre"),
            c => w.extend(digit_word(c)),
        }
    }
    w.join(" ")
}

/// Whole thousands / hundreds ("3500" → "three thousand five hundred",
/// "11000" → "one one thousand"); other values digit by digit.
pub fn alt_words(ft: u32) -> String {
    let th = ft / 1000;
    let hu = (ft % 1000) / 100;
    if ft % 100 != 0 || ft == 0 {
        return digits(ft);
    }
    let mut w = Vec::new();
    if th > 0 {
        w.push(digits(th));
        w.push("thousand".into());
    }
    if hu > 0 {
        w.push(DIGITS[hu as usize].into());
        w.push("hundred".into());
    }
    w.join(" ")
}

/// Flight level number: digits, but whole hundreds as "one hundred".
pub fn fl_words(fl: u32) -> String {
    if fl % 100 == 0 && fl > 0 { format!("{} hundred", DIGITS[(fl / 100) as usize % 10]) } else { digits(fl) }
}

/// Frequency (kHz) as shown: "118.5", "121.975", "118.505".
pub fn freq_text(khz: u32) -> String {
    let s = format!("{}.{:03}", khz / 1000, khz % 1000);
    let t = s.trim_end_matches('0');
    if t.ends_with('.') { format!("{t}0") } else { t.to_string() }
}

/// Frequency spoken: "one one eight decimal five".
pub fn freq_words(khz: u32) -> String {
    let t = freq_text(khz);
    let (a, b) = t.split_once('.').unwrap_or((&t, "0"));
    format!("{} decimal {}", spell(a), spell(b))
}

/// Runway ident as shown ("27R").
pub fn runway_text(id: &str) -> String {
    id.to_ascii_uppercase()
}

impl Utt {
    pub fn new() -> Utt {
        Utt::default()
    }

    fn push_text(&mut self, shown: &str) {
        if shown.is_empty() {
            return;
        }
        if !self.text.is_empty() && !self.text.ends_with(' ') {
            self.text.push(' ');
        }
        // Capitalise the first word of a sentence.
        if self.text.is_empty() || self.text.trim_end().ends_with('.') {
            let mut c = shown.chars();
            if let Some(f) = c.next() {
                self.text.extend(f.to_uppercase());
                self.text.push_str(c.as_str());
                return;
            }
        }
        self.text.push_str(shown);
    }

    fn push_spoken(&mut self, spoken: &str) {
        let v = vocab();
        let lower = spoken.to_lowercase();
        let words: Vec<&str> = lower.split_whitespace().collect();
        let mut i = 0;
        while i < words.len() {
            let mut hit = None;
            for n in (1..=v.max_words.min(words.len() - i)).rev() {
                let key = words[i..i + n].join(" ");
                if let Some(ix) = v.get(&key) {
                    hit = Some((ix, n));
                    break;
                }
            }
            match hit {
                Some((ix, n)) => {
                    self.toks.push(Tok::Word(ix));
                    i += n;
                }
                None => {
                    self.missing.push(words[i].to_string());
                    i += 1;
                }
            }
        }
    }

    /// Say a phrase and show it as written.
    pub fn say(&mut self, s: &str) -> &mut Self {
        self.push_text(s);
        self.push_spoken(s);
        self
    }

    /// Say `spoken`, show `shown`.
    pub fn say_as(&mut self, spoken: &str, shown: &str) -> &mut Self {
        self.push_text(shown);
        self.push_spoken(spoken);
        self
    }

    /// Text only (no sound), e.g. a symbol.
    pub fn show(&mut self, shown: &str) -> &mut Self {
        self.push_text(shown);
        self
    }

    pub fn comma(&mut self) -> &mut Self {
        let t = self.text.trim_end().to_string();
        if !t.is_empty() && !t.ends_with(',') && !t.ends_with('.') {
            self.text = t + ",";
        }
        self.toks.push(Tok::Gap(GAP_COMMA));
        self
    }

    pub fn stop(&mut self) -> &mut Self {
        let t = self.text.trim_end().trim_end_matches(',').to_string();
        if !t.is_empty() && !t.ends_with('.') {
            self.text = t + ".";
        }
        self.toks.push(Tok::Gap(GAP_STOP));
        self
    }

    pub fn gap(&mut self, s: f32) -> &mut Self {
        self.toks.push(Tok::Gap(s));
        self
    }

    /// Digits of a number, shown as the number.
    pub fn num(&mut self, n: u32) -> &mut Self {
        self.say_as(&digits(n), &n.to_string())
    }

    pub fn runway(&mut self, id: &str) -> &mut Self {
        self.say_as(&format!("runway {}", runway_words(id)), &format!("runway {}", runway_text(id)))
    }

    pub fn freq(&mut self, khz: u32) -> &mut Self {
        self.say_as(&freq_words(khz), &freq_text(khz))
    }

    /// "heading 270" (three digits, true heading in degrees).
    pub fn heading(&mut self, deg: f32) -> &mut Self {
        let h = heading_value(deg);
        self.say_as(&format!("heading {}", spell(&format!("{h:03}"))), &format!("heading {h:03}"))
    }

    /// An altitude / level: "altitude 3,000 feet" below the transition level
    /// `tl` (flight level number), else "flight level 80".
    pub fn level(&mut self, ft: f32, tl: u32) -> &mut Self {
        let ft = round_alt(ft);
        if ft >= tl * 100 {
            let fl = ft / 100;
            self.say_as(&format!("flight level {}", fl_words(fl)), &format!("flight level {fl}"))
        } else {
            self.say_as(&format!("altitude {} feet", alt_words(ft)), &format!("altitude {} feet", thousands(ft)))
        }
    }

    /// "2,500 feet" (no "altitude").
    pub fn feet(&mut self, ft: f32) -> &mut Self {
        let ft = round_alt(ft);
        self.say_as(&format!("{} feet", alt_words(ft)), &format!("{} feet", thousands(ft)))
    }

    /// "QNH 1013".
    pub fn qnh(&mut self, hpa: f32) -> &mut Self {
        let q = hpa.round().clamp(900.0, 1099.0) as u32;
        self.say_as(&format!("qnh {}", digits(q)), &format!("QNH {q}"))
    }

    pub fn squawk(&mut self, code: u32) -> &mut Self {
        let s = format!("{code:04}");
        self.say_as(&format!("squawk {}", spell(&s)), &format!("squawk {s}"))
    }

    /// Surface wind: "240 degrees 12 knots", "calm", "variable 3 knots", with gusts.
    pub fn wind(&mut self, from: Option<f32>, kt: f32, gust: Option<f32>) -> &mut Self {
        let k = kt.round() as u32;
        if k < 1 {
            return self.say("calm");
        }
        match from {
            Some(d) => {
                let d = ((d / 10.0).round() as u32 * 10).rem_euclid(360);
                let d = if d == 0 { 360 } else { d };
                self.say_as(&format!("{} degrees", spell(&format!("{d:03}"))), &format!("{d:03} degrees"));
            }
            None => {
                self.say("variable");
            }
        }
        self.say_as(&format!("{} knots", digits(k)), &format!("{k} knots"));
        if let Some(g) = gust.filter(|g| *g >= kt + 9.5) {
            let g = g.round() as u32;
            self.say_as(&format!("gusting {} knots", digits(g)), &format!("gusting {g} knots"));
        }
        self
    }

    /// Spell a code (taxiway "A10", stand "512", ICAO "EGKK").
    pub fn spell(&mut self, code: &str) -> &mut Self {
        self.say_as(&spell(code), code)
    }

    pub fn is_empty(&self) -> bool {
        self.toks.is_empty()
    }

    /// Drop the trailing pause and tidy the subtitle.
    pub fn finish(mut self) -> Utt {
        while matches!(self.toks.last(), Some(Tok::Gap(_))) {
            self.toks.pop();
        }
        let t = self.text.trim().trim_end_matches(',').to_string();
        self.text = if t.ends_with('.') || t.ends_with('?') || t.is_empty() { t } else { t + "." };
        self
    }
}

/// Heading 1..=360 in whole degrees (0 → 360).
pub fn heading_value(deg: f32) -> u32 {
    let h = (deg.rem_euclid(360.0) / 5.0).round() as u32 * 5 % 360;
    if h == 0 { 360 } else { h }
}

fn round_alt(ft: f32) -> u32 {
    ((ft / 100.0).round().max(0.0) * 100.0) as u32
}

/// "3,500".
pub fn thousands(n: u32) -> String {
    if n >= 1000 { format!("{},{:03}", n / 1000, n % 1000) } else { n.to_string() }
}

/// A radio callsign: full (initial contact) and abbreviated forms.
#[derive(Clone, Debug, PartialEq)]
pub struct Callsign {
    pub full_text: String,
    pub full_spoken: String,
    pub short_text: String,
    pub short_spoken: String,
}

impl Callsign {
    /// An airline / military telephony designator and a flight number.
    pub fn flight(telephony: &str, number: u32) -> Callsign {
        let t = format!("{} {number}", capitalise(telephony));
        let s = format!("{} {}", telephony.to_lowercase(), digits(number));
        Callsign { full_text: t.clone(), full_spoken: s.clone(), short_text: t, short_spoken: s }
    }
    /// A registration ("G-SKYW" → "Golf Sierra Kilo Yankee Whiskey", abbreviated
    /// "Golf Yankee Whiskey" — first character and last two).
    pub fn registration(reg: &str) -> Callsign {
        let c: String = reg.chars().filter(|c| c.is_ascii_alphanumeric()).collect();
        let short: String = c.chars().take(1).chain(c.chars().skip(c.len().saturating_sub(2))).collect();
        let short_shown = format!("{}-{}", &short[..1], &short[1..]);
        Callsign { full_text: reg.to_string(), full_spoken: spell(&c), short_text: short_shown, short_spoken: spell(&short) }
    }
    /// The callsign the player gets for an aircraft type.
    pub fn for_type(id: &str, category: crate::flight::Category) -> Callsign {
        match id {
            "lancer" => Callsign::flight("Viper", 11),
            "hercules" | "vulcan" => Callsign::flight("Ascot", 412),
            "galaxy" => Callsign::flight("Reach", 471),
            _ => match category {
                crate::flight::Category::Airliner => Callsign::flight("Skyward", 247),
                crate::flight::Category::FastJet => Callsign::flight("Viper", 11),
                _ => Callsign::registration("G-SKYW"),
            },
        }
    }
}

fn capitalise(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().chain(c.flat_map(|x| x.to_lowercase())).collect(),
        None => String::new(),
    }
}

/// Spoken names of airports / radar units in the vocabulary, by ICAO code.
pub const NAMES: &[(&str, &str, &str)] = &[
    ("EGLL", "heathrow", "Heathrow"),
    ("EGKK", "gatwick", "Gatwick"),
    ("EGSS", "stansted", "Stansted"),
    ("EGGW", "luton", "Luton"),
    ("EGLC", "london city", "London City"),
    ("EGKB", "biggin", "Biggin"),
    ("EGLF", "farnborough", "Farnborough"),
    ("EGWU", "northolt", "Northolt"),
    ("EGMC", "southend", "Southend"),
    ("EGCC", "manchester", "Manchester"),
    ("EGBB", "birmingham", "Birmingham"),
    ("EGGD", "bristol", "Bristol"),
    ("EGPH", "edinburgh", "Edinburgh"),
    ("EGPF", "glasgow", "Glasgow"),
    ("EGSC", "cambridge", "Cambridge"),
    ("EGTK", "oxford", "Oxford"),
    ("EGHI", "southampton", "Southampton"),
    ("EGHH", "bournemouth", "Bournemouth"),
    ("EGKA", "shoreham", "Shoreham"),
    ("EGKR", "redhill", "Redhill"),
    ("EGTR", "elstree", "Elstree"),
    ("EGMD", "lydd", "Lydd"),
    ("EGMH", "manston", "Manston"),
    ("EGSH", "norwich", "Norwich"),
    ("EGNX", "east midlands", "East Midlands"),
    ("EGGP", "liverpool", "Liverpool"),
    ("EGNT", "newcastle", "Newcastle"),
    ("EGFF", "cardiff", "Cardiff"),
    ("EGTE", "exeter", "Exeter"),
    ("EGJJ", "jersey", "Jersey"),
    ("EGJB", "guernsey", "Guernsey"),
    ("EGAA", "belfast", "Belfast"),
    ("EIDW", "dublin", "Dublin"),
    ("LFPG", "paris", "Paris"),
    ("LFPO", "orly", "Orly"),
    ("EHAM", "schiphol", "Schiphol"),
    ("EBBR", "brussels", "Brussels"),
    ("EDDF", "frankfurt", "Frankfurt"),
    ("LOWI", "innsbruck", "Innsbruck"),
    ("LSZH", "zurich", "Zurich"),
    ("LSGG", "geneva", "Geneva"),
    ("EGVN", "brize", "Brize"),
    ("EGUL", "lakenheath", "Lakenheath"),
    ("EGUN", "mildenhall", "Mildenhall"),
    ("EGYM", "marham", "Marham"),
    ("EGXC", "coningsby", "Coningsby"),
    ("EGXW", "waddington", "Waddington"),
    ("EGXP", "scampton", "Scampton"),
];

/// Radar units named in frequency descriptions ("THAMES RDR").
pub const UNITS: &[(&str, &str, &str)] = &[("THAMES", "thames", "Thames"), ("LONDON", "london", "London")];

/// Approach units whose published name the data lacks: (airport, unit name key).
pub const APPROACH_UNITS: &[(&str, &str)] = &[("EGLC", "THAMES"), ("EGKB", "THAMES")];

/// Spoken / shown name of an airport (None: not in the vocabulary).
pub fn airport_name(icao: &str) -> Option<(&'static str, &'static str)> {
    NAMES.iter().find(|n| n.0.eq_ignore_ascii_case(icao)).map(|n| (n.1, n.2))
}
