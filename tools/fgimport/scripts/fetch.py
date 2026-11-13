#!/usr/bin/env python3
"""Fetch the files a FlightGear model needs (model XML tree, .ac, textures) + FDM files.

usage: fetch.py <id> <source> <aircraft-dir-name> <entry...>
  source: svn:<Name>  -> FGAddon trunk/Aircraft/<Name>
          gh:<owner>/<repo>@<branch>
Files land in n/src/<id>/<relpath>. Entries are paths relative to the aircraft root.
Prints a manifest (n/src/<id>/_fetched.txt) and skipped external references.
"""
import os, re, sys, urllib.request, urllib.error

ROOT = os.environ.get("FG_SRC", "fgsrc")
UA = os.environ.get("FG_UA", "skyward-flightsim/0.3 (fgimport)")
TEX = (".png", ".rgb", ".rgba", ".sgi", ".jpg", ".jpeg", ".dds", ".bw")
SKIP = [p for p in os.environ.get("SKIP", "").split(",") if p]

def main():
    ident, source, acname, *entries = sys.argv[1:]
    if source.startswith("svn:"):
        base = "https://svn.code.sf.net/p/flightgear/fgaddon/trunk/Aircraft/" + source[4:] + "/"
    else:
        repo, br = source[3:].split("@")
        base = f"https://raw.githubusercontent.com/{repo}/{br}/"
    out = os.path.join(ROOT, ident)
    os.makedirs(out, exist_ok=True)
    prefix = "Aircraft/" + acname + "/"
    seen, ext, missing = set(), set(), set()
    queue = list(entries)

    def norm(p):
        parts = []
        for s in p.replace("\\", "/").split("/"):
            if s in ("", "."):
                continue
            if s == "..":
                if parts:
                    parts.pop()
                continue
            parts.append(s)
        return "/".join(parts)

    def resolve(ref, cur_dir, kind):
        ref = ref.strip()
        if not ref or "$" in ref or "{" in ref:
            return None
        if ref.startswith(prefix):
            return [norm(ref[len(prefix):])]
        if ref.startswith("Aircraft/") or ref.startswith("Models/") and kind == "xml" and False:
            ext.add(ref)
            return None
        if ref.startswith("/"):
            ext.add(ref)
            return None
        # relative: file dir first, then aircraft root
        return [norm(cur_dir + "/" + ref), norm(ref)]

    def fetch(rel):
        dst = os.path.join(out, rel)
        if os.path.exists(dst):
            return True
        url = base + urllib.request.quote(rel)
        try:
            req = urllib.request.Request(url, headers={"User-Agent": UA})
            data = urllib.request.urlopen(req, timeout=300).read()
        except urllib.error.HTTPError:
            return False
        except Exception as e:
            print("ERR", rel, e)
            return False
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(dst, "wb") as f:
            f.write(data)
        return True

    def want(cands):
        for c in cands:
            if any(s in c for s in SKIP):
                return
            if c in seen:
                return
        for c in cands:
            if c in seen:
                return
            if fetch(c):
                seen.add(c)
                queue.append(c)
                return
        missing.add(cands[0])

    for e in entries:
        if fetch(e):
            seen.add(e)
        else:
            missing.add(e)
    while queue:
        rel = queue.pop()
        path = os.path.join(out, rel)
        d = os.path.dirname(rel)
        low = rel.lower()
        if low.endswith(".xml"):
            txt = open(path, encoding="utf-8", errors="replace").read()
            txt = re.sub(r"<!--.*?-->", "", txt, flags=re.S)
            refs = re.findall(r'include="([^"]+)"', txt)
            refs += re.findall(r'file="([^"]+)"', txt)
            refs += [r for r in re.findall(r"<path>([^<]+)</path>", txt)]
            refs += re.findall(r"<texture>([^<]+)</texture>", txt)
            refs += re.findall(r"<image>([^<]+)</image>", txt)
            refs += re.findall(r"<inherits-from>([^<]+)</inherits-from>", txt)
            for r in refs:
                r = r.strip()
                rl = r.lower()
                if rl.endswith(".xml") or rl.endswith(".ac") or rl.endswith(TEX) or rl.endswith(".eff"):
                    c = resolve(r, d, "xml")
                elif "inherits" in r or not os.path.splitext(r)[1]:
                    # JSBSim file="name" without extension (engines / systems) or effects
                    c = resolve(r + ".xml", d, "xml")
                    if c:
                        c = c + [norm("Engines/" + r + ".xml"), norm("Systems/" + r + ".xml")]
                    c2 = resolve(r + ".eff", d, "xml")
                    if c2:
                        want(c2) if False else None
                else:
                    continue
                if c:
                    want(c)
        elif low.endswith(".ac"):
            txt = open(path, encoding="latin-1").read()
            for t in set(re.findall(r'^texture "([^"]+)"', txt, flags=re.M)):
                c = resolve(t, d, "ac")
                if c:
                    want(c)
        elif low.endswith(".eff"):
            txt = open(path, encoding="utf-8", errors="replace").read()
            for r in re.findall(r"<(?:image|inherits-from)>([^<]+)<", txt):
                r = r.strip()
                if r.lower().endswith(TEX) or r.lower().endswith(".eff"):
                    c = resolve(r, d, "xml")
                elif not os.path.splitext(r)[1]:
                    c = resolve(r + ".eff", d, "xml")
                else:
                    c = None
                if c:
                    want(c)
    tot = 0
    with open(os.path.join(out, "_fetched.txt"), "w") as f:
        for s in sorted(seen):
            sz = os.path.getsize(os.path.join(out, s))
            tot += sz
            f.write(f"{sz}\t{s}\n")
        for m in sorted(missing):
            f.write(f"MISSING\t{m}\n")
        for e in sorted(ext):
            f.write(f"EXTERNAL\t{e}\n")
    print(f"{ident}: {len(seen)} files, {tot/1e6:.1f} MB, {len(missing)} missing, {len(ext)} external")

main()
