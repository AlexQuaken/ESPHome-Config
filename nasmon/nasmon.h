// NAS monitor screen "Rack" (variant 01) for esp32-c6-lcd (ST7789 320x172).
// Pixel port of the JS mockup https://claude.ai/artifact/2YiQusGoKRmgGTVMdn4SAZ (same fonts, same coordinates).
// Data: JSON from nasmon.py on the DS1019+ (http://192.168.0.7:8099/nas.json), parsed by nasmon_parse().
#pragma once
#define NASMON_SCREEN 1
#define CUSTOM_SCREEN 1
#include "esphome.h"
#include "esphome/components/json/json_util.h"
#include <string>
#include <vector>
#include <cmath>

namespace nasmon {
using esphome::Color;
using esphome::display::Display;

struct Vol { std::string id, type, eta; float used = 0, tot = 1; int rd = 0, wr = 0; float sync = -1; };
struct Disk { std::string bay, sn; int temp = 0; bool io = false, ok = true; };
struct Data {
  bool valid = false; uint32_t got_ms = 0;
  std::string time = "--:--:--", up = "--", fan = "--";
  bool fanOk = true;
  int cpu = 0, cores[4] = {0, 0, 0, 0}, cpuTemp = 0, ram = 0, rd = 0, wr = 0, frigate = 0, cams = 0;
  float load = 0, ramUsed = 0, ramTot = 0, rx = 0, tx = 0;
  std::vector<Vol> vols; std::vector<Disk> disks; std::vector<int> nvme;
  std::vector<float> h_cpu, h_rx, h_wr;
};
inline Data &D() { static Data d; return d; }

inline void parse(const std::string &body) {
  esphome::json::parse_json(body, [](JsonObject r) -> bool {
    Data &d = D();
    d.time = r["time"] | "--:--:--"; d.up = r["up"] | "--"; d.fan = r["fan"] | "--"; d.fanOk = r["fanOk"] | true;
    d.cpu = r["cpu"] | 0; d.cpuTemp = r["cpuTemp"] | 0; d.ram = r["ram"] | 0;
    d.load = r["load"] | 0.0f; d.ramUsed = r["ramUsed"] | 0.0f; d.ramTot = r["ramTot"] | 0.0f;
    d.rx = r["rx"] | 0.0f; d.tx = r["tx"] | 0.0f; d.rd = r["rd"] | 0; d.wr = r["wr"] | 0;
    d.frigate = r["frigate"] | 0; d.cams = r["cams"] | 0;
    JsonArray c = r["cores"]; for (int i = 0; i < 4; i++) d.cores[i] = c[i] | 0;
    d.vols.clear();
    for (JsonObject v : r["vols"].as<JsonArray>()) {
      Vol x; x.id = v["id"] | ""; x.type = v["type"] | ""; x.eta = v["eta"] | "";
      x.used = v["used"] | 0.0f; x.tot = v["tot"] | 1.0f; x.rd = v["rd"] | 0; x.wr = v["wr"] | 0;
      x.sync = v["sync"].isNull() ? -1.0f : (float) (v["sync"] | 0.0f);
      d.vols.push_back(x);
    }
    d.disks.clear();
    for (JsonObject v : r["disks"].as<JsonArray>()) {
      Disk x; x.bay = v["bay"] | ""; x.sn = v["sn"] | ""; x.temp = v["temp"] | 0; x.io = v["io"] | false; x.ok = v["ok"] | true;
      d.disks.push_back(x);
    }
    d.nvme.clear(); for (JsonObject v : r["nvme"].as<JsonArray>()) d.nvme.push_back(v["temp"] | 0);
    auto arr = [&](const char *k, std::vector<float> &out) { out.clear(); for (float f : r[k].as<JsonArray>()) out.push_back(f); };
    arr("h_cpu", d.h_cpu); arr("h_rx", d.h_rx); arr("h_wr", d.h_wr);
    d.valid = true; d.got_ms = esphome::millis();
    return true;
  });
}

// ---- palette (same hex as the mockup) ----
inline Color hex(uint32_t h) { return Color((h >> 16) & 255, (h >> 8) & 255, h & 255); }
inline Color mix(uint32_t a, uint32_t b, float t) {
  auto ch = [&](int s) { float x = ((a >> s) & 255), y = ((b >> s) & 255); return (uint8_t) std::lround(x + (y - x) * t); };
  return Color(ch(16), ch(8), ch(0));
}
const uint32_t BG = 0x07070e, PANEL = 0x14121f, SLOT = 0x191529, LINE = 0x2a2540, LINE2 = 0x3b3458, INK = 0xf3effd,
               MUTED = 0xe2ddf6, DIM = 0xc2bbe0, CYAN = 0x22d3ee, VIO = 0xa855f7, IND = 0x6366f1, MAG = 0xec4899,
               OK = 0x2ee6a8, WARN = 0xf5b32e, BAD = 0xf43f5e;

struct G {
  Display &it; esphome::font::Font *f6, *f16, *f32;
  void rect(int x, int y, int w, int h, Color c) { if (w > 0 && h > 0) it.filled_rectangle(x, y, w, h, c); }
  void hline(int x, int y, int w, Color c) { rect(x, y, w, 1, c); }
  void vline(int x, int y, int h, Color c) { rect(x, y, 1, h, c); }
  // f: 0 = 6x8, 1 = 8x16, 2 = 16x32; al: 'l' or 'r'. Width counted in characters (UTF-8 aware), like the mockup.
  int text(int x, int y, const std::string &s, Color c, int f = 0, char al = 'l') {
    int n = 0; for (unsigned char ch : s) if ((ch & 0xC0) != 0x80) n++;
    int cw = f == 0 ? 6 : f == 1 ? 8 : 16, w = n * cw;
    if (al == 'r') x -= w;
    it.print(x, y, f == 0 ? f6 : f == 1 ? f16 : f32, c, esphome::display::TextAlign::TOP_LEFT, s.c_str());
    return w;
  }
  void bar(int x, int y, int w, int h, float pct, Color c, Color bg) {
    rect(x, y, w, h, bg); int f = (int) std::lround(w * std::max(0.0f, std::min(100.0f, pct)) / 100.0f); rect(x, y, f, h, c);
  }
  void vbar(int x, int y, int w, int h, float pct, Color c, Color bg) {
    rect(x, y, w, h, bg); int f = (int) std::lround(h * std::max(0.0f, std::min(100.0f, pct)) / 100.0f); rect(x, y + h - f, w, f, c);
  }
  void line(int x0, int y0, int x1, int y1, Color c) { it.line(x0, y0, x1, y1, c); }
  // same algorithm as gfx.spark(): whole history squeezed into w pixels
  void spark(int x, int y, int w, int h, const std::vector<float> &a, Color c, float lo, float hi, bool autohi, bool fill, Color fc) {
    if (a.size() < 2 || w < 2 || h < 2) return;
    if (autohi) { hi = a[0]; for (float v : a) hi = std::max(hi, v); }
    if (hi - lo < 1e-6f) hi = lo + 1;
    int prev = -1;
    for (int i = 0; i < w; i++) {
      float v = a[(size_t) std::floor((double) i * (a.size() - 1) / (w - 1))];
      v = std::max(lo, std::min(hi, v));
      int py = y + h - 1 - (int) std::lround((v - lo) / (hi - lo) * (h - 1));
      if (fill) vline(x + i, py + 1, y + h - 1 - py, fc);
      if (prev >= 0) line(x + i - 1, prev, x + i, py, c); else it.draw_pixel_at(x + i, py, c);
      prev = py;
    }
  }
};

inline std::string fmtT(float v) { char b[16]; if (v < 1) snprintf(b, sizeof b, "%dG", (int) std::lround(v * 1024)); else snprintf(b, sizeof b, "%.1fT", v); return b; }
inline std::string mbs(float v) { char b[16]; if (v >= 100) snprintf(b, sizeof b, "%d", (int) std::lround(v)); else snprintf(b, sizeof b, "%.1f", v); return b; }
inline std::string io(float n) { char b[16]; if (n < 1000) snprintf(b, sizeof b, "%d", (int) std::lround(n)); else snprintf(b, sizeof b, "%.1fk", n / 1000); return b; }
inline int pct(const Vol &v) { return (int) std::lround(std::max(0.0f, std::min(100.0f, v.used / v.tot * 100.0f))); }
inline uint32_t loadCol(int v) { return v >= 85 ? BAD : v >= 60 ? WARN : OK; }
inline uint32_t tempCol(int v) { return v >= 85 ? BAD : v >= 70 ? WARN : OK; }
inline uint32_t diskCol(int t) { return t >= 50 ? BAD : t >= 45 ? WARN : OK; }
inline uint32_t vcol(const std::string &id) { return id == "V1" ? CYAN : id == "V2" ? VIO : id == "V3" ? IND : MAG; }
inline uint32_t useCol(const Vol &v) { int p = pct(v); return p >= 95 ? BAD : p >= 85 ? WARN : vcol(v.id); }
inline std::string i2s(int v) { return std::to_string(v); }

inline void draw(Display &it, esphome::font::Font *f6, esphome::font::Font *f16, esphome::font::Font *f32, const std::string &clk) {
  G g{it, f6, f16, f32};
  Data &D = nasmon::D();
  it.fill(hex(BG));
  // header
  g.rect(0, 0, 320, 11, hex(PANEL)); g.hline(0, 11, 320, hex(LINE2));
  g.text(4, 2, "DS1019+", hex(INK));
  bool stale = !D.valid || esphome::millis() - D.got_ms > 10000;
  const std::string &now = clk.empty() ? D.time : clk;
  if (stale) {
    g.text(52, 2, D.valid ? "NO DATA FROM NAS" : "WAITING FOR NAS", hex(BAD));
    g.text(316, 2, now, hex(DIM), 0, 'r');
    if (!D.valid) return;
  } else {
    g.text(52, 2, "UP " + D.up, hex(MUTED));
    g.text(124, 2, "FAN " + D.fan, hex(D.fanOk ? MUTED : BAD));
    g.text(316, 2, now, hex(INK), 0, 'r');
  }

  // left: cpu
  g.text(4, 15, "CPU", hex(MUTED));
  g.text(74, 15, "J3455", hex(DIM));
  g.text(4, 24, i2s(D.cpu) + "%", hex(loadCol(D.cpu)), 2);
  for (int i = 0; i < 4; i++) g.vbar(74 + i * 10, 24, 7, 32, D.cores[i], hex(loadCol(D.cores[i])), hex(SLOT));
  g.text(118, 24, i2s(D.cpuTemp) + "°", hex(tempCol(D.cpuTemp)), 1);
  g.text(118, 44, "TEMP", hex(DIM));
  g.text(4, 62, "RAM", hex(MUTED));
  g.bar(28, 63, 72, 6, D.ram, hex(D.ram >= 85 ? BAD : CYAN), hex(SLOT));
  { char b[24]; snprintf(b, sizeof b, "%.1f/%gG", D.ramUsed, D.ramTot); g.text(154, 62, b, hex(INK), 0, 'r'); }
  g.text(4, 74, "LOAD", hex(MUTED));
  { char b[16]; snprintf(b, sizeof b, "%.2f", D.load); g.text(34, 74, b, hex(INK)); }
  g.text(4, 86, "NET", hex(MUTED));
  g.text(28, 86, "↓" + mbs(D.rx), hex(CYAN));
  g.text(70, 86, "↑" + mbs(D.tx), hex(VIO));
  g.spark(112, 86, 42, 8, D.h_rx, hex(CYAN), 0, 0, true, true, mix(CYAN, BG, 0.75f));
  g.text(4, 98, "DISK", hex(MUTED));
  g.text(34, 98, "R " + io(D.rd), hex(OK));
  g.text(70, 98, "W " + io(D.wr), hex(WARN));
  g.spark(112, 98, 42, 8, D.h_wr, hex(WARN), 0, 0, true, true, mix(WARN, BG, 0.75f));
  g.vline(158, 14, 106, hex(LINE));

  // right: volumes
  for (size_t i = 0; i < D.vols.size() && i < 4; i++) {
    const Vol &v = D.vols[i]; int y = 14 + (int) i * 27;
    g.text(162, y, v.id, hex(vcol(v.id)));
    g.text(180, y, v.type, hex(MUTED));
    g.text(316, y, fmtT(v.used) + "/" + fmtT(v.tot), hex(INK), 0, 'r');
    g.bar(162, y + 10, 154, 5, pct(v), hex(useCol(v)), hex(SLOT));
    if (v.sync >= 0) {
      g.rect(162, y + 15, (int) std::lround(154 * v.sync / 100), 1, hex(WARN));
      char b[24]; snprintf(b, sizeof b, "SYNC %.1f%%", v.sync);
      g.text(162, y + 17, b, hex(WARN));
      g.text(316, y + 17, "ETA " + v.eta, hex(WARN), 0, 'r');
    } else {
      g.text(162, y + 17, "R " + io(v.rd), hex(v.rd ? OK : DIM));
      g.text(204, y + 17, "W " + io(v.wr), hex(v.wr ? WARN : DIM));
      g.text(316, y + 17, i2s(pct(v)) + "%", hex(MUTED), 0, 'r');
    }
  }

  // bottom: temperatures of bays 1..5 and both NVMe
  g.hline(0, 124, 320, hex(LINE));
  int i = 0;
  for (const Disk &d : D.disks) {
    int x = 3 + i++ * 45;
    g.text(x, 128, "B" + d.bay, hex(DIM));
    g.text(x + 14, 128, i2s(d.temp) + "°", hex(diskCol(d.temp)));
    g.rect(x + 36, 130, 4, 4, hex(!d.ok ? BAD : d.io ? OK : SLOT));
  }
  for (size_t k = 0; k < D.nvme.size(); k++) {
    int x = 3 + i++ * 45;
    g.text(x, 128, "N" + i2s((int) k), hex(DIM));
    g.text(x + 14, 128, i2s(D.nvme[k]) + "°", hex(diskCol(D.nvme[k])));
    g.rect(x + 36, 130, 4, 4, hex(SLOT));
  }
  g.hline(0, 140, 320, hex(LINE));
  g.text(4, 144, "CPU 60S", hex(DIM));
  g.text(166, 144, "NET ↓ 60S", hex(DIM));
  g.spark(4, 154, 150, 15, D.h_cpu, hex(CYAN), 0, 100, false, true, mix(CYAN, BG, 0.7f));
  g.spark(166, 154, 150, 15, D.h_rx, hex(VIO), 0, 0, true, true, mix(VIO, BG, 0.7f));
}
}  // namespace nasmon

namespace nasmon {
inline esphome::font::Font *&F6() { static esphome::font::Font *f = nullptr; return f; }
inline esphome::font::Font *&F16() { static esphome::font::Font *f = nullptr; return f; }
inline esphome::font::Font *&F32() { static esphome::font::Font *f = nullptr; return f; }
// called from the device on_boot: the shared package lambda must not reference font ids
inline void set_fonts(esphome::font::Font *a, esphome::font::Font *b, esphome::font::Font *c) { F6() = a; F16() = b; F32() = c; }
}  // namespace nasmon

// hook used by packages/ble_radar.yaml (#ifdef CUSTOM_SCREEN): screen mode 2 is the NAS monitor
namespace custom_screen {
inline bool draw(esphome::display::Display &it, int mode, esphome::ESPTime tn) {
  if (mode != 2 || nasmon::F6() == nullptr) return false;
  char clk[12] = "";
  if (tn.is_valid()) snprintf(clk, sizeof(clk), "%02d:%02d:%02d", tn.hour, tn.minute, tn.second);
  nasmon::draw(it, nasmon::F6(), nasmon::F16(), nasmon::F32(), clk);
  return true;
}
}  // namespace custom_screen
