//! FlightGear PropertyList XML: a small owned element tree with `include=`
//! merging and `alias=` resolution.

use std::path::{Path, PathBuf};

#[derive(Clone, Debug, Default)]
pub struct Elem {
    pub name: String,
    pub attrs: Vec<(String, String)>,
    pub text: String,
    pub kids: Vec<Elem>,
}

impl Elem {
    pub fn attr(&self, k: &str) -> Option<&str> {
        self.attrs.iter().find(|(a, _)| a == k).map(|(_, v)| v.as_str())
    }
    /// Last child with this name (overlays from includes come last and win).
    pub fn child(&self, n: &str) -> Option<&Elem> {
        self.kids.iter().rev().find(|k| k.name == n)
    }
    pub fn children<'a>(&'a self, n: &'a str) -> impl Iterator<Item = &'a Elem> + 'a {
        self.kids.iter().filter(move |k| k.name == n)
    }
    pub fn text_of(&self, n: &str) -> Option<&str> {
        self.child(n).map(|e| e.text.trim())
    }
    pub fn f(&self, n: &str) -> Option<f32> {
        self.text_of(n).and_then(|t| t.parse::<f32>().ok())
    }
    /// Child by '/'-separated path.
    pub fn path(&self, p: &str) -> Option<&Elem> {
        let mut e = self;
        for seg in p.split('/').filter(|s| !s.is_empty()) {
            let (name, idx) = match seg.split_once('[') {
                Some((n, i)) => (n, i.trim_end_matches(']').parse::<usize>().unwrap_or(0)),
                None => (seg, 0),
            };
            e = e.kids.iter().filter(|k| k.name == name).nth(idx)?;
        }
        Some(e)
    }
}

fn convert(n: roxmltree::Node) -> Elem {
    let mut e = Elem { name: n.tag_name().name().to_string(), ..Default::default() };
    for a in n.attributes() {
        e.attrs.push((a.name().to_string(), a.value().to_string()));
    }
    for c in n.children() {
        if c.is_element() {
            e.kids.push(convert(c));
        } else if c.is_text() {
            e.text.push_str(c.text().unwrap_or(""));
        }
    }
    e
}

pub struct Ctx {
    /// Aircraft root directory on disk.
    pub root: PathBuf,
    /// "Aircraft/<name>/" prefix used inside the files.
    pub prefix: String,
}

impl Ctx {
    /// Resolve a path reference made from a file in `dir`.
    pub fn resolve(&self, r: &str, dir: &Path) -> Option<PathBuf> {
        let r = r.trim();
        if r.is_empty() {
            return None;
        }
        if let Some(rest) = r.strip_prefix(&self.prefix) {
            let p = self.root.join(rest);
            return p.exists().then_some(p);
        }
        if r.starts_with("Aircraft/") || r.starts_with('/') {
            return None;
        }
        let p = dir.join(r);
        if p.exists() {
            return Some(p);
        }
        let p = self.root.join(r);
        p.exists().then_some(p)
    }
}

pub fn load(ctx: &Ctx, path: &Path) -> Result<Elem, String> {
    load_depth(ctx, path, 0)
}

fn load_depth(ctx: &Ctx, path: &Path, depth: usize) -> Result<Elem, String> {
    if depth > 12 {
        return Err("include depth".into());
    }
    let raw = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut text = String::from_utf8_lossy(&raw).into_owned();
    if text.starts_with('\u{feff}') {
        text.remove(0);
    }
    let doc = roxmltree::Document::parse_with_options(&text, roxmltree::ParsingOptions { allow_dtd: true, ..Default::default() })
        .map_err(|e| format!("{}: {e}", path.display()))?;
    let mut root = convert(doc.root_element());
    let dir = path.parent().unwrap_or(Path::new("."));
    expand_includes(ctx, &mut root, dir, depth);
    resolve_aliases(&mut root);
    Ok(root)
}

fn expand_includes(ctx: &Ctx, e: &mut Elem, dir: &Path, depth: usize) {
    for k in &mut e.kids {
        expand_includes(ctx, k, dir, depth);
    }
    if let Some(inc) = e.attr("include").map(|s| s.to_string()) {
        if let Some(p) = ctx.resolve(&inc, dir) {
            if let Ok(base) = load_depth(ctx, &p, depth + 1) {
                let mut kids = base.kids;
                kids.append(&mut e.kids);
                e.kids = kids;
                if e.text.trim().is_empty() {
                    e.text = base.text;
                }
            }
        }
    }
}

/// FG `<overlay>`: merge `o` over `e` (children matched by name and
/// occurrence; leaves replace the text, new nodes are appended).
pub fn overlay(e: &mut Elem, o: &Elem) {
    let mut seen: std::collections::HashMap<&str, usize> = Default::default();
    for k in &o.kids {
        let n = seen.entry(k.name.as_str()).or_default();
        let nth = *n;
        *n += 1;
        match e.kids.iter_mut().filter(|c| c.name == k.name).nth(nth) {
            Some(t) if !k.kids.is_empty() => overlay(t, k),
            Some(t) => {
                t.text = k.text.clone();
                // An overlaid value replaces an alias.
                t.attrs.retain(|(a, _)| a != "alias");
            }
            None => e.kids.push(k.clone()),
        }
    }
}

pub fn resolve_aliases(root: &mut Elem) {
    let snapshot = root.clone();
    fn walk(e: &mut Elem, snap: &Elem) {
        if let Some(a) = e.attr("alias") {
            let mut p = a.trim();
            while let Some(r) = p.strip_prefix("../") {
                p = r;
            }
            let p = p.trim_start_matches('/');
            if let Some(t) = snap.path(p) {
                e.text = t.text.clone();
                if e.kids.is_empty() {
                    e.kids = t.kids.clone();
                }
            }
        }
        for k in &mut e.kids {
            walk(k, snap);
        }
    }
    walk(root, &snapshot);
}

fn esc(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;")
}

impl Elem {
    /// Serialise (comments dropped, text trimmed).
    pub fn to_xml(&self, out: &mut String, depth: usize) {
        let ind = "  ".repeat(depth);
        out.push_str(&format!("{ind}<{}", self.name));
        for (k, v) in &self.attrs {
            if k == "include" {
                continue;
            }
            out.push_str(&format!(" {k}=\"{}\"", esc(v).replace('"', "&quot;")));
        }
        let t = self.text.trim();
        if self.kids.is_empty() {
            if t.is_empty() {
                out.push_str("/>\n");
            } else if t.contains('\n') {
                out.push_str(&format!(">\n{}\n{ind}</{}>\n", esc(t), self.name));
            } else {
                out.push_str(&format!(">{}</{}>\n", esc(t), self.name));
            }
            return;
        }
        out.push_str(">\n");
        if !t.is_empty() {
            out.push_str(&format!("{ind}  {}\n", esc(t)));
        }
        for k in &self.kids {
            k.to_xml(out, depth + 1);
        }
        out.push_str(&format!("{ind}</{}>\n", self.name));
    }
}
