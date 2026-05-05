#!/usr/bin/env python3

import math
from pathlib import Path


# ---------------------------------------------------------------------------
# Dosya okuma / yazma
# ---------------------------------------------------------------------------

def parse_waypoints_file(filepath: str) -> tuple[str, list[dict]]:
    """
    Standart ArduPilot .waypoints dosyasını okur.
    Geri döner: (header_satırı, waypoint_listesi)
    """
    path = Path(filepath)
    if not path.exists():
        raise FileNotFoundError(f"Dosya bulunamadı: {filepath}")

    with path.open("r", encoding="utf-8") as f:
        lines = f.readlines()

    if not lines:
        raise ValueError("Dosya boş.")

    header = lines[0].strip()   # örn. "QGC WPL 110"
    waypoints: list[dict] = []

    for lineno, line in enumerate(lines[1:], start=2):
        line = line.strip()
        if not line:
            continue
        parts = line.split("\t")
        if len(parts) < 12:
            print(f"  [UYARI] Satır {lineno} beklenen sütun sayısına sahip değil, atlandı.")
            continue

        waypoints.append({
            "index":       int(parts[0]),
            "current":     int(parts[1]),
            "frame":       int(parts[2]),
            "command":     int(parts[3]),
            "param1":      float(parts[4]),
            "param2":      float(parts[5]),
            "param3":      float(parts[6]),
            "param4":      float(parts[7]),
            "lat":         float(parts[8]),
            "lon":         float(parts[9]),
            "alt":         float(parts[10]),
            "autocontinue": int(parts[11]),
        })

    return header, waypoints


def write_waypoints_file(filepath: str, header: str, waypoints: list[dict]) -> None:
    """Waypoint listesini ArduPilot .waypoints formatında yazar."""
    with open(filepath, "w", encoding="utf-8", newline="\n") as f:
        f.write(header + "\n")
        for i, wp in enumerate(waypoints):
            f.write(
                f"{i}\t"
                f"{wp['current']}\t"
                f"{wp['frame']}\t"
                f"{wp['command']}\t"
                f"{wp['param1']:.6f}\t"
                f"{wp['param2']:.6f}\t"
                f"{wp['param3']:.6f}\t"
                f"{wp['param4']:.6f}\t"
                f"{wp['lat']:.8f}\t"
                f"{wp['lon']:.8f}\t"
                f"{wp['alt']:.3f}\t"
                f"{wp['autocontinue']}\n"
            )


# ---------------------------------------------------------------------------
# Koordinat dönüşümleri  (WGS84 ↔ yerel ENU metre)
# ---------------------------------------------------------------------------

_R_EARTH = 6_371_000.0   # metre


def latlon_to_enu(lat: float, lon: float, ref_lat: float, ref_lon: float) -> tuple[float, float]:
    """
    Enlem/boylam → yerel ENU (East-North) metre koordinatları.
    Referans nokta orijin kabul edilir.
    """
    x = math.radians(lon - ref_lon) * math.cos(math.radians(ref_lat)) * _R_EARTH
    y = math.radians(lat - ref_lat) * _R_EARTH
    return x, y


def enu_to_latlon(x: float, y: float, ref_lat: float, ref_lon: float) -> tuple[float, float]:
    """Yerel ENU (East-North) metre koordinatları → enlem/boylam."""
    lat = ref_lat + math.degrees(y / _R_EARTH)
    lon = ref_lon + math.degrees(x / (_R_EARTH * math.cos(math.radians(ref_lat))))
    return lat, lon


# ---------------------------------------------------------------------------
# 8 şekli üretici
# ---------------------------------------------------------------------------

def generate_figure8(
    c1_lat: float, c1_lon: float,
    c2_lat: float, c2_lon: float,
    alt: float,
    radius: float,
    n_points: int,
    start_circle: int,
    direction: str,
    laps: int,
) -> list[tuple[float, float, float]]:
    """
    Kum saati (hourglass) şeklinde waypoint üretir.

    Geometri:
      • C1, C2 : iki çemberin merkezleri
      • M      : C1-C2 orta noktası → X kavşak / geçiş noktası
      • A, B   : C1 çemberinin yarım çember uçları  (C1-C2 eksenine dik)
      • C, D   : C2 çemberinin yarım çember uçları

    Her tur:
      A ──yarım çember──► B ──düz──► M ──düz──► C ──yarım çember──► D ──düz──► M ──düz──► A

      B→M→C ve D→M→A her zaman geometrik olarak doğrusaldır (M ikisi üzerinde de yatar).

    Lob başına waypoint dağılımı (n_half = n_points // 2):
      • n_half - 1  yay noktası  (A dahil, B dahil, eşit aralıklı 180°)
      • 1           M geçiş noktası
      Toplam per lob: n_half   →   per tur: n_points ✓

    Dönüş yönü:
      direction='+' → birinci lob CCW (dışa doğru şişer), ikinci lob CW
      direction='-' → birinci lob CW  (içe doğru şişer), ikinci lob CCW
    """
    # ── Referans: M (orta nokta) ──────────────────────────────────────────
    ref_lat = (c1_lat + c2_lat) / 2.0
    ref_lon = (c1_lon + c2_lon) / 2.0

    c1x, c1y = latlon_to_enu(c1_lat, c1_lon, ref_lat, ref_lon)
    c2x, c2y = latlon_to_enu(c2_lat, c2_lon, ref_lat, ref_lon)

    r = radius
    if r < 0.5:
        raise ValueError(f"radius çok küçük ({r:.2f} m). En az 0.5 m girin.")

    dist = math.hypot(c2x - c1x, c2y - c1y)
    if dist < 0.1:
        raise ValueError("İki merkez waypoint birbirine çok yakın. Daha uzak iki WP seçin.")

    # ── Eksen ve dik vektör ───────────────────────────────────────────────
    # axis: C1→C2 birim vektör
    # perp: eksene dik birim vektör (axis'i 90° CCW döndürülmüş)
    ax = (c2x - c1x) / dist
    ay = (c2y - c1y) / dist
    perp_x = -ay          # CCW 90°
    perp_y =  ax
    theta_perp = math.atan2(perp_y, perp_x)

    # ── Lob sırası ────────────────────────────────────────────────────────
    # A ve C her zaman theta_perp açısındaki nokta, B ve D = theta_perp ± π
    # start_circle=1 → önce C1 çemberi, sonra C2
    # start_circle=2 → önce C2 çemberi, sonra C1
    if start_circle == 1:
        circles = [(c1x, c1y), (c2x, c2y)]
    else:
        circles = [(c2x, c2y), (c1x, c1y)]

    # İlk lobun dönüş yönü; ikinci lob her zaman ters
    first_sign = 1 if direction == "+" else -1
    signs = [first_sign, -first_sign]

    # ── Nokta sayıları ────────────────────────────────────────────────────
    n_half = n_points // 2      # lob başına toplam waypoint
    n_arc  = n_half - 1         # yay noktası sayısı (A ve B dahil, M hariç)
    # n_arc >= 3 garantili çünkü n_points >= 8 → n_half >= 4 → n_arc >= 3

    waypoints: list[tuple[float, float, float]] = []

    for _lap in range(laps):
        for (cx, cy), sign in zip(circles, signs):
            # ── Yarım çember: A'dan (theta_perp) B'ye (theta_perp + sign*π) ──
            # k = 0 → A (giriş),  k = n_arc-1 → B (çıkış)
            for k in range(n_arc):
                angle = theta_perp + sign * math.pi * k / (n_arc - 1)
                wpx = cx + r * math.cos(angle)
                wpy = cy + r * math.sin(angle)
                lat, lon = enu_to_latlon(wpx, wpy, ref_lat, ref_lon)
                waypoints.append((lat, lon, alt))

            # ── M: X kavşak / geçiş noktası ──────────────────────────────
            # B→M→C (veya D→M→A) her zaman doğrusaldır: M her iki doğru üzerinde yatar.
            waypoints.append((ref_lat, ref_lon, alt))

    return waypoints


# ---------------------------------------------------------------------------
# Yardımcı: şablon waypoint oluştur
# ---------------------------------------------------------------------------

def make_wp(ref: dict, lat: float, lon: float, alt: float) -> dict:
    """
    Referans bir waypoint'in çerçeve ve bayrak ayarlarını koruyarak
    yeni bir NAV_WAYPOINT (komut=16) üretir.
    """
    return {
        "index":        0,          # sonradan yeniden numaralandırılır
        "current":      0,
        "frame":        ref["frame"],
        "command":      16,         # MAV_CMD_NAV_WAYPOINT
        "param1":       0.0,
        "param2":       0.0,
        "param3":       0.0,
        "param4":       0.0,
        "lat":          lat,
        "lon":          lon,
        "alt":          alt,
        "autocontinue": 1,
    }


# ---------------------------------------------------------------------------
# Ana fonksiyon — parametreleri buradan düzenle ve çalıştır
# ---------------------------------------------------------------------------

def run(
    file:         str,        # Giriş .waypoints dosyasının yolu
    index:        int,        # Mission Planner WP numarası (0-tabanlı, home=0)
                              #   → bu WP ve bir sonraki (index+1) çember merkezi olur
    radius:       float,      # Çember yarıçapı, METRE cinsinden (kullanıcı belirler)
    points:       int,        # Tam bir tur için üretilecek waypoint sayısı (çift, >= 8)
    start_circle: int,        # İlk dönülecek çember: 1 → index'teki WP, 2 → (index+1)'deki WP
    direction:    str,        # Birinci çemberin yönü: "+" = CCW (saat tersi), "-" = CW (saat yönü)
                              #   → ikinci çember her zaman tersine döner
    laps:         int,        # Tur sayısı (her iki çember = 1 tam tur)
    output:       str = "",   # Çıkış dosyası yolu; boş bırakılırsa giriş dosyasının üzerine yazar
) -> None:

    # --- Doğrulama ---
    if points < 8 or points % 2 != 0:
        raise ValueError("points çift bir tam sayı ve en az 8 olmalıdır.")
    if laps < 1:
        raise ValueError("laps en az 1 olmalıdır.")
    if start_circle not in (1, 2):
        raise ValueError("start_circle 1 veya 2 olmalıdır.")
    if direction not in ("+", "-"):
        raise ValueError('direction "+" veya "-" olmalıdır.')
    if radius <= 0:
        raise ValueError("radius pozitif bir sayı olmalıdır.")

    # --- Dosya yükleme ---
    print(f"\n[YÜKLEME] {file}")
    header, waypoints = parse_waypoints_file(file)
    n_wp = len(waypoints)
    print(f"  → {n_wp} waypoint okundu.")

    # --- İndeks kontrolü ---
    # index, Mission Planner'ın gösterdiği WP numarasıdır (0-tabanlı, home=0).
    c1_pos = index
    c2_pos = index + 1
    if c1_pos < 0 or c2_pos >= n_wp:
        raise IndexError(
            f"index={index}: dosyada {n_wp} waypoint var (WP#0..WP#{n_wp-1}). "
            f"Her iki merkez (WP#{index} ve WP#{index+1}) dosyada bulunmalıdır."
        )

    c1 = waypoints[c1_pos]
    c2 = waypoints[c2_pos]

    # --- Bilgi çıktısı ---
    print(f"\n[PARAMETRELER]")
    print(f"  Merkez 1 (WP #{index})   : lat={c1['lat']:.7f}, lon={c1['lon']:.7f}, alt={c1['alt']:.1f} m")
    print(f"  Merkez 2 (WP #{index+1}) : lat={c2['lat']:.7f}, lon={c2['lon']:.7f}, alt={c2['alt']:.1f} m")
    print(f"  Yarıçap                  : {radius} m")
    print(f"  Başlangıç çemberi        : {start_circle}")
    print(f"  Birinci çember yönü      : {'CCW (saat yönü tersi)' if direction == '+' else 'CW (saat yönü)'}")
    print(f"  İkinci çember yönü       : {'CW (saat yönü)' if direction == '+' else 'CCW (saat yönü tersi)'}")
    print(f"  Tur sayısı               : {laps}")
    print(f"  Nokta / tur              : {points}  (lob başı {points // 2}: 1 M + {points // 2 - 1} çember)")
    print(f"  Toplam üretilecek        : {points * laps} waypoint")

    # --- 8 / kum saati şekli üretimi ---
    alt = (c1["alt"] + c2["alt"]) / 2.0

    generated_coords = generate_figure8(
        c1_lat=c1["lat"], c1_lon=c1["lon"],
        c2_lat=c2["lat"], c2_lon=c2["lon"],
        alt=alt,
        radius=radius,
        n_points=points,
        start_circle=start_circle,
        direction=direction,
        laps=laps,
    )

    gen_wps = [make_wp(c1, lat, lon, a) for lat, lon, a in generated_coords]

    # --- Yeni waypoint listesi ---
    before = waypoints[:c1_pos]
    after  = waypoints[c2_pos + 1:]

    new_waypoints = before + gen_wps + after

    # Sadece sıra numaralarını yeniden yaz
    for i, wp in enumerate(new_waypoints):
        wp["index"] = i

    # --- Dosyaya yaz ---
    output_path = output if output else file
    write_waypoints_file(output_path, header, new_waypoints)

    print(f"\n[TAMAMLANDI]")
    print(f"  Merkez öncesi waypoint : {len(before)}")
    print(f"  Kum saati waypoint      : {len(gen_wps)}")
    print(f"  Merkez sonrası waypoint : {len(after)}")
    print(f"  Toplam                  : {len(new_waypoints)} waypoint")
    print(f"  Çıkış dosyası          : {output_path}\n")


# ===========================================================================
#  BURADAN DÜZENLE VE ÇALIŞTIR
# ===========================================================================
if __name__ == "__main__":
    run(
        file         = "input.waypoints",  # giriş dosyası yolu
        index        = 5,                    # Mission Planner WP numarası (0-tabanlı, home=0)
        radius       = 10.0,                 # çember yarıçapı, METRE  ← yeni parametre
        points       = 12,                   # tur başına toplam waypoint (çift, >= 8)
        start_circle = 1,                    # 1 → index'teki WP önce,  2 → (index+1)'deki WP önce
        direction    = "+",                  # "+" = CCW (saat tersi),  "-" = CW (saat yönü)
        laps         = 1,                    # tur sayısı
        output       = "output.waypoints",   # çıkış dosyası ("" → giriş dosyasının üzerine yazar)
    )