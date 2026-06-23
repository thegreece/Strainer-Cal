/**
 * strainer-calc.js  — VRF H/R Strainer Sizing · Calculation Engine v2.0
 * =========================================================================
 * Pure calculation module (no DOM). Attach to window for use in index.html.
 *
 * PHYSICS OVERVIEW
 * ─────────────────
 * 1. Pipe velocity          V_pipe = Q_v / A_pipe
 * 2. Screen face velocity   V_scr  = Q_v / A_free
 *    where A_free = π × D_scr × L_scr × FAR   (cylindrical element)
 *          D_scr  ≈ 0.90 × pipe_ID             (Y-strainer approximation)
 *          FAR    = free area ratio of mesh
 * 3. Pressure drop (velocity-head method)
 *    ΔP_body   = K_body × ρ × V_pipe² / 2   (body form loss)
 *    ΔP_screen = K_mesh × ρ × V_scr²  / 2   (screen resistance)
 *    ΔP_total  = ΔP_body + ΔP_screen         (kPa)
 * 4. Reverse calc: given strainer spec → solve for max Q_v, max capacity
 *    Governing limit = min(screen limit, pipe-velocity limit)
 */

/* ── ACR Copper Tube (OD × wall, mm) ─────────────────────────────── */
const PIPES = {
  mm:[
    {od:6.35,  wall:0.76,  label:'6.35 mm  (¼")' },
    {od:9.52,  wall:0.89,  label:'9.52 mm  (⅜")' },
    {od:12.7,  wall:0.89,  label:'12.7 mm  (½")' },
    {od:15.88, wall:1.02,  label:'15.88 mm (⅝")' },
    {od:19.05, wall:1.07,  label:'19.05 mm (¾")' },
    {od:22.22, wall:1.14,  label:'22.22 mm (⅞")' },
    {od:25.4,  wall:1.14,  label:'25.4 mm  (1")'  },
    {od:28.58, wall:1.27,  label:'28.58 mm (1⅛")'},
    {od:31.75, wall:1.27,  label:'31.75 mm (1¼")'},
    {od:34.93, wall:1.40,  label:'34.93 mm (1⅜")'},
    {od:38.1,  wall:1.40,  label:'38.1 mm  (1½")'},
    {od:41.28, wall:1.52,  label:'41.28 mm (1⅝")'},
    {od:44.45, wall:1.65,  label:'44.45 mm (1¾")'}
  ],
  in:[
    {od:0.250, wall:0.030, label:'¼"  (6.35 mm)' },
    {od:0.375, wall:0.035, label:'⅜"  (9.52 mm)' },
    {od:0.500, wall:0.035, label:'½"  (12.7 mm)'  },
    {od:0.625, wall:0.040, label:'⅝"  (15.88 mm)'},
    {od:0.750, wall:0.042, label:'¾"  (19.05 mm)'},
    {od:0.875, wall:0.045, label:'⅞"  (22.22 mm)'},
    {od:1.000, wall:0.045, label:'1"  (25.4 mm)'  },
    {od:1.125, wall:0.050, label:'1⅛" (28.58 mm)'},
    {od:1.250, wall:0.050, label:'1¼" (31.75 mm)'},
    {od:1.375, wall:0.055, label:'1⅜" (34.93 mm)'},
    {od:1.500, wall:0.055, label:'1½" (38.1 mm)' },
    {od:1.625, wall:0.060, label:'1⅝" (41.28 mm)'},
    {od:1.750, wall:0.065, label:'1¾" (44.45 mm)'}
  ]
};

/* ── Mesh Database ────────────────────────────────────────────────── */
/* Tyler/US standard mesh · stainless wire cloth (304/316 SS)
   FAR (Free Area Ratio) = open area / total area  (typical values)
   K_clean = screen loss coefficient (velocity head at V_scr)
   Ref: Crane TP-410, manufacturer data (Spirax, Watts, Apollo)       */
const MESH_DB = {
  20:  {opening_um: 850, FAR:0.560, K_clean:2.0,  label:'20 mesh  — 850 μm'},
  30:  {opening_um: 590, FAR:0.510, K_clean:3.0,  label:'30 mesh  — 590 μm'},
  40:  {opening_um: 420, FAR:0.455, K_clean:4.5,  label:'40 mesh  — 420 μm'},
  60:  {opening_um: 250, FAR:0.385, K_clean:6.5,  label:'60 mesh  — 250 μm'},
  80:  {opening_um: 180, FAR:0.335, K_clean:8.5,  label:'80 mesh  — 180 μm'},
  100: {opening_um: 149, FAR:0.300, K_clean:11.0, label:'100 mesh — 149 μm'},
  120: {opening_um: 125, FAR:0.275, K_clean:13.5, label:'120 mesh — 125 μm'},
};
const MESH_KEYS = [20,30,40,60,80,100,120];

/* Max allowable screen-face velocity (conservative, clean screen)
   Based on: V_scr_max to keep ΔP_screen < 10 kPa at clean condition  */
const V_SCR_MAX = { liquid:0.25, suction:2.5, discharge:2.0 }; // m/s

/* Recommended mesh per line (primary, alternate) */
const REC_MESH = {
  liquid:    {primary:100, alt:80,  reason:'Protect EEV/TXV — fine mesh required'},
  suction:   {primary:60,  alt:40,  reason:'Protect compressor — medium mesh'},
  discharge: {primary:40,  alt:30,  reason:'High-temp HP gas — coarser mesh, lower ΔP'},
};

/* Strainer body type by pipe OD */
function getStrainerType(od_mm) {
  if (od_mm <= 12.7)  return 'In-line filter/drier (Sporlan, Danfoss FSI)';
  if (od_mm <= 28.58) return 'Y-strainer (SS screen, solder/flare end)';
  if (od_mm <= 41.28) return 'Y-strainer or Basket strainer';
  return 'Basket strainer (welded/flanged)';
}

/* ── Refrigerant Properties (simplified linear ≈ REFPROP ±3%) ────── */
/* Valid: Tc 30–65°C · Te -10–20°C · subcooling/superheat not modeled  */
function getRefProps(ref, Tc, Te) {
  let h_liq_c, h_liq_e, h_vap_e, h_vap_c;
  let rho_liq, rho_suc, rho_dis;

  if (ref === 'R410A') {
    h_liq_c = 200 + 1.580 * Tc;
    h_liq_e = 200 + 1.580 * Te;
    h_vap_e = 423 + 0.400 * Te;
    h_vap_c = 423 + 0.400 * Tc;
    rho_liq = Math.max(800, 1170 - 3.78 * Tc);
    rho_suc = Math.max(4.0,  6.50 * Math.exp(0.058 * Te));
    rho_dis = Math.max(8.0,  6.50 * Math.exp(0.058 * Tc));
  } else {                                       // R32
    h_liq_c = 200 + 3.360 * Tc;
    h_liq_e = 200 + 3.360 * Te;
    h_vap_e = 521 + 0.240 * Te;
    h_vap_c = 521 + 0.240 * Tc;
    rho_liq = Math.max(700, 1050 - 4.00 * Tc);
    rho_suc = Math.max(3.0,  4.80 * Math.exp(0.065 * Te));
    rho_dis = Math.max(6.0,  4.80 * Math.exp(0.065 * Tc));
  }

  return {
    dh_cool: Math.max(1, h_vap_e - h_liq_c), // kJ/kg · refrigerating effect
    dh_heat: Math.max(1, h_vap_c - h_liq_e), // kJ/kg · heating effect
    rho_liq,   // kg/m³  liquid at Tc (liquid line strainer)
    rho_suc,   // kg/m³  suction vapor at Te
    rho_dis,   // kg/m³  discharge vapor at Tc
  };
}

/* ── Pipe Cross-Section ──────────────────────────────────────────── */
function getPipeGeom(od_mm, wall_mm) {
  const id_mm = od_mm - 2 * wall_mm;
  const id_m  = id_mm / 1000;
  const A     = Math.PI * id_m * id_m / 4;
  return { id_mm, id_m, A };
}

/* ── Pipe-Line Flow Calculation ──────────────────────────────────── */
function calcPipeLine(cap_kw, dh, rho, K_body, od_mm, wall_mm) {
  if (!cap_kw || cap_kw <= 0 || od_mm <= 0) return null;
  const { id_mm, A } = getPipeGeom(od_mm, wall_mm);
  const mdot     = cap_kw / dh;            // kg/s
  const Qv       = mdot / rho;             // m³/s
  const V_pipe   = Qv / A;                 // m/s
  const dP_body  = K_body * rho * V_pipe * V_pipe / 2 / 1000; // kPa
  return { mdot, Qv, Qv_L: Qv * 60000, V_pipe, dP_body, id_mm };
}

/* ── Screen Element Geometry ──────────────────────────────────────── */
/* Cylindrical screen: D_scr ≈ 0.90 × pipe_ID  (Y-strainer element)   */
function getScreenGeom(od_mm, wall_mm, meshSize, screenLen_mm) {
  const mesh  = MESH_DB[meshSize];
  if (!mesh) return null;
  const { id_mm } = getPipeGeom(od_mm, wall_mm);
  const D_scr = id_mm * 0.90 / 1000;                      // m
  const L_scr = screenLen_mm / 1000;                       // m
  const A_free = Math.PI * D_scr * L_scr * mesh.FAR;      // m²
  return { D_scr_mm: D_scr * 1000, L_scr_mm: screenLen_mm, A_free, mesh };
}

/* ── Minimum Screen Length ────────────────────────────────────────── */
/* Solve: V_scr_max × π × D_scr × L × FAR = Q_v  →  L_min            */
function calcMinScreenLength(Qv_m3s, od_mm, wall_mm, meshSize, lineType) {
  const mesh  = MESH_DB[meshSize];
  if (!mesh || Qv_m3s <= 0) return 0;
  const V_max  = V_SCR_MAX[lineType] || 2.5;
  const { id_mm } = getPipeGeom(od_mm, wall_mm);
  const D_scr  = id_mm * 0.90 / 1000;
  const L_min  = Qv_m3s / (V_max * Math.PI * D_scr * mesh.FAR);
  // Also enforce: L ≥ 1.5 × pipe_ID (practical fabrication minimum)
  const L_prac = id_mm * 1.5 / 1000;
  return Math.max(L_min, L_prac) * 1000; // mm
}

/* ── Screen Face Velocity & ΔP ────────────────────────────────────── */
function calcScreen(Qv_m3s, od_mm, wall_mm, meshSize, screenLen_mm, rho, K_body, V_pipe) {
  const geo = getScreenGeom(od_mm, wall_mm, meshSize, screenLen_mm);
  if (!geo) return null;
  const V_scr    = geo.A_free > 0 ? Qv_m3s / geo.A_free : Infinity;
  const dp_body  = K_body             * rho * V_pipe * V_pipe / 2 / 1000;
  const dp_screen = geo.mesh.K_clean  * rho * V_scr  * V_scr  / 2 / 1000;
  const dp_total = dp_body + dp_screen;
  return { ...geo, V_scr, dp_body, dp_screen, dp_total };
}

/* ── Reverse Calculation ──────────────────────────────────────────── */
/* Given strainer specs → max flow rate & max system capacity           */
function reverseCalc(od_mm, wall_mm, meshSize, screenLen_mm, lineType, rho, dh) {
  const mesh = MESH_DB[meshSize];
  if (!mesh) return null;
  const V_scr_max  = V_SCR_MAX[lineType] || 2.5;
  const V_pipe_max = lineType === 'liquid' ? 1.5 : 15.0;

  const { id_mm, A: A_pipe } = getPipeGeom(od_mm, wall_mm);
  const D_scr  = id_mm * 0.90 / 1000;
  const L_scr  = screenLen_mm / 1000;
  const A_free = Math.PI * D_scr * L_scr * mesh.FAR;

  const Qv_scr  = V_scr_max  * A_free;   // limited by screen velocity
  const Qv_pipe = V_pipe_max * A_pipe;   // limited by pipe velocity
  const Qv_max  = Math.min(Qv_scr, Qv_pipe);
  const constraint = Qv_scr <= Qv_pipe ? 'screen' : 'pipe';

  const V_pipe_at = Qv_max / A_pipe;
  const V_scr_at  = A_free > 0 ? Qv_max / A_free : 0;
  const mdot_max  = Qv_max * rho;
  const cap_max   = mdot_max * dh;

  return {
    Qv_scr, Qv_pipe, Qv_max,
    Qv_L_max: Qv_max * 60000,
    constraint,
    V_pipe_at, V_scr_at,
    mdot_max, cap_max,
    A_free,
  };
}

/* ── Status Helpers ───────────────────────────────────────────────── */
const LIM = {
  pipe_v:   { liquid:{ok:1.5, wa:2.5}, gas:{ok:15.0, wa:20.0} },
  scr_v:    { liquid:{ok:0.25,wa:0.35}, suction:{ok:2.5,wa:3.5}, discharge:{ok:2.0,wa:3.0} },
  dp_body:  { ok:20, wa:30 },
  dp_total: { ok:25, wa:40 },
};

function pipeVSt(V, lt)   { const l=lt==='liquid'?LIM.pipe_v.liquid:LIM.pipe_v.gas;  return V<=l.ok?'ok':V<=l.wa?'wa':'er'; }
function scrVSt(V, lt)    { const l=LIM.scr_v[lt]||LIM.scr_v.suction;               return V<=l.ok?'ok':V<=l.wa?'wa':'er'; }
function dpBodySt(dp)     { return dp<=LIM.dp_body.ok?'ok':dp<=LIM.dp_body.wa?'wa':'er'; }
function dpTotalSt(dp)    { return dp<=LIM.dp_total.ok?'ok':dp<=LIM.dp_total.wa?'wa':'er'; }
function worstSt(...args) { return args.includes('er')?'er':args.includes('wa')?'wa':'ok'; }
function stLabel(s)       { return s==='ok'?'OK ✓':s==='wa'?'Warning':'Over limit'; }

/* ── Expose to global scope ──────────────────────────────────────── */
Object.assign(window, {
  PIPES, MESH_DB, MESH_KEYS, V_SCR_MAX, REC_MESH,
  getRefProps, getPipeGeom, calcPipeLine, getScreenGeom,
  calcMinScreenLength, calcScreen, reverseCalc, getStrainerType,
  pipeVSt, scrVSt, dpBodySt, dpTotalSt, worstSt, stLabel, LIM,
});
