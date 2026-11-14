/* Earth detector serial protocol: COBS framing, CRC16, frame decoders, command encoders.
 * Works in the browser (global `EDProto`) and in Node (module.exports).
 *
 * Wire format: COBS(type u8, seq u16, payload..., crc16 u16) followed by 0x00. Little-endian.
 * CRC16-CCITT (init 0xFFFF, poly 0x1021) over type..payload.
 * Reference implementation: tools/monitor.py and firmware/src/protocol.h.
 */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.EDProto = factory();
})(typeof globalThis !== 'undefined' ? globalThis : this, function () {
  'use strict';

  const T = {
    DATA: 0x01, STATUS: 0x02, EVENT: 0x03, HEALTH: 0x04, LOG: 0x05, PARAMS: 0x06, HELLO: 0x07, RAW: 0x10,
    SET_PARAM: 0x80, REZERO: 0x81, RAW_MODE: 0x82, TEST_ALARM: 0x83, SILENCE: 0x84,
    SET_CAL: 0x85, GET_PARAMS: 0x86, RESET_PARAMS: 0x87
  };
  const TYPE_NAMES = {};
  Object.keys(T).forEach(k => { TYPE_NAMES[T[k]] = k.toLowerCase(); });

  const STATES = ['INIT', 'IDLE', 'CANDIDATE', 'ALARM', 'FAULT'];
  const CHECKS = ['duration', 'steady', 'freq', 'crest', 'periodic', 'base'];
  // Parameter ids, in firmware order (params.h). `unit` is informational.
  const PARAM_NAMES = ['on_thr', 'off_thr', 'confirm_s', 'fmin', 'fmax', 'crest_max', 'periodic_min',
    'decay_min', 'base_ratio_min', 'latch_s', 'quiet_s', 'buzzer_en', 'lvl1', 'lvl2', 'lvl3', 'lvl4',
    'lvl5', 'buzzer_mode'];
  const P = {};
  PARAM_NAMES.forEach((n, i) => { P[n] = i; });
  const OUT_SCALE = 2000; // counts per g in DATA frames (also sent in HELLO)

  function crc16(d, start, end) {
    let crc = 0xFFFF;
    start = start || 0;
    end = end === undefined ? d.length : end;
    for (let i = start; i < end; i++) {
      crc ^= d[i] << 8;
      for (let k = 0; k < 8; k++) crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
    }
    return crc;
  }

  // Decode one COBS block (without the trailing 0x00). Returns Uint8Array or null if malformed.
  function cobsDecode(d) {
    const out = new Uint8Array(d.length);
    let o = 0, i = 0;
    while (i < d.length) {
      const code = d[i++];
      if (code === 0 || i + code - 1 > d.length) return null;
      for (let k = 1; k < code; k++) out[o++] = d[i++];
      if (code !== 0xFF && i < d.length) out[o++] = 0;
    }
    return out.subarray(0, o);
  }

  function cobsEncode(d) {
    const out = new Uint8Array(d.length + Math.ceil(d.length / 254) + 2);
    let o = 1, codeIdx = 0, code = 1;
    for (let i = 0; i < d.length; i++) {
      if (d[i] === 0) {
        out[codeIdx] = code; codeIdx = o++; code = 1;
      } else {
        out[o++] = d[i]; code++;
        if (code === 0xFF) { out[codeIdx] = code; codeIdx = o++; code = 1; }
      }
    }
    out[codeIdx] = code;
    return out.subarray(0, o);
  }

  // Build a complete wire frame (COBS + 0x00 delimiter).
  function encodeFrame(type, seq, payload) {
    payload = payload || new Uint8Array(0);
    const body = new Uint8Array(3 + payload.length + 2);
    body[0] = type; body[1] = seq & 0xFF; body[2] = (seq >> 8) & 0xFF;
    body.set(payload, 3);
    const c = crc16(body, 0, body.length - 2);
    body[body.length - 2] = c & 0xFF; body[body.length - 1] = c >> 8;
    const enc = cobsEncode(body);
    const out = new Uint8Array(enc.length + 1);
    out.set(enc, 0); out[enc.length] = 0;
    return out;
  }

  // ---------------------------------------------------------------- commands (host -> device)
  function mk(type, len, fill) {
    const p = new Uint8Array(len);
    if (fill) fill(new DataView(p.buffer), p);
    return encodeFrame(type, 0, p);
  }
  const Cmd = {
    setParam: (id, v) => mk(T.SET_PARAM, 5, dv => { dv.setUint8(0, id); dv.setFloat32(1, v, true); }),
    rezero: mask => mk(T.REZERO, 1, dv => dv.setUint8(0, mask === undefined ? 3 : mask)),
    rawMode: on => mk(T.RAW_MODE, 1, dv => dv.setUint8(0, on ? 1 : 0)),
    testAlarm: () => mk(T.TEST_ALARM, 0),
    silence: () => mk(T.SILENCE, 0),
    setCal: (sensor, off, gain) => mk(T.SET_CAL, 25, dv => {
      dv.setUint8(0, sensor);
      for (let k = 0; k < 3; k++) { dv.setFloat32(1 + 4 * k, off[k], true); dv.setFloat32(13 + 4 * k, gain[k], true); }
    }),
    getParams: () => mk(T.GET_PARAMS, 0),
    resetParams: () => mk(T.RESET_PARAMS, 0)
  };

  // ---------------------------------------------------------------- payload decoders (device -> host)
  function decodeData(dv, p) {
    if (p.length < 15) return null;
    const sensor = dv.getUint8(0), flags = dv.getUint8(1), n = dv.getUint8(2);
    if (p.length < 15 + n * 12) return null;
    const s = new Int16Array(n * 6);
    for (let i = 0; i < n * 6; i++) s[i] = dv.getInt16(15 + 2 * i, true);
    return {
      type: 'data', sensor, flags, gap: !!(flags & 1), zeroed: !!(flags & 2), n,
      idx: dv.getUint32(3, true), tUs: dv.getUint32(7, true), fs: dv.getFloat32(11, true),
      samples: s // n x [ax ay az fx fy fz], counts of 1/OUT_SCALE g
    };
  }
  function decodeRaw(dv, p) {
    if (p.length < 6) return null;
    const sensor = dv.getUint8(0), n = dv.getUint8(1);
    if (p.length < 6 + n * 6) return null;
    const s = new Int16Array(n * 3);
    for (let i = 0; i < n * 3; i++) s[i] = dv.getInt16(6 + 2 * i, true);
    return { type: 'raw', sensor, n, idx: dv.getUint32(2, true), samples: s };
  }
  function decodeStatus(dv, p) {
    if (p.length < 45) return null;
    const f = i => dv.getFloat32(5 + 4 * i, true);
    return {
      type: 'status', state: dv.getUint8(0), level: dv.getUint8(1), flags: dv.getUint8(2),
      silenced: !!(dv.getUint8(2) & 1), ok1: !!(dv.getUint8(2) & 2), ok2: !!(dv.getUint8(2) & 4),
      rawMode: !!(dv.getUint8(2) & 8),
      passMask: dv.getUint8(3), evalMask: dv.getUint8(4),
      rms: f(0), peak: f(1), crest: f(2), freq: f(3), freqQ: f(4), periodicity: f(5), decay: f(6),
      baseRms: f(7), basePeak: f(8), dur: f(9)
    };
  }
  function decodeEvent(dv, p) {
    if (p.length < 56) return null;
    const f = i => dv.getFloat32(8 + 4 * i, true);
    return {
      type: 'event', id: dv.getUint32(0, true), verdict: dv.getUint8(4), accepted: dv.getUint8(4) !== 0,
      passMask: dv.getUint8(5), evalMask: dv.getUint8(6), level: dv.getUint8(7),
      dur: f(0), peak: f(1), rms: f(2), freq: f(3), crest: f(4), periodicity: f(5), decay: f(6),
      baseRatio: f(7), pga: f(8), mmi: f(9), disp: f(10), tStart: dv.getUint32(52, true)
    };
  }
  function decodeHealth(dv, p) {
    const SZ = 39;
    if (p.length < 14 + 2 * SZ) return null;
    const h = { type: 'health', uptimeMs: dv.getUint32(0, true), dropped: dv.getUint32(4, true),
      heap: dv.getUint32(8, true), vbat: dv.getUint16(12, true), sensors: [] };
    for (let i = 0; i < 2; i++) {
      const o = 14 + i * SZ;
      h.sensors.push({
        present: !!dv.getUint8(o), devFirst: dv.getUint8(o + 1), devLast: dv.getUint8(o + 2),
        selftest: dv.getUint8(o + 3),
        stDelta: [dv.getInt16(o + 4, true), dv.getInt16(o + 6, true), dv.getInt16(o + 8, true)],
        odr: dv.getFloat32(o + 10, true), raw: dv.getUint32(o + 14, true),
        overruns: dv.getUint32(o + 18, true), i2c: dv.getUint32(o + 22, true),
        reinits: dv.getUint32(o + 26, true), noiseMg: dv.getFloat32(o + 30, true),
        tiltDeg: dv.getFloat32(o + 34, true), zeroed: !!dv.getUint8(o + 38)
      });
    }
    return h;
  }
  function decodeParams(dv, p) {
    if (p.length < 1) return null;
    const n = Math.min(dv.getUint8(0), Math.floor((p.length - 1) / 4));
    const values = [];
    for (let i = 0; i < n; i++) values.push(dv.getFloat32(1 + 4 * i, true));
    return { type: 'params', n, values };
  }
  function decodeHello(dv, p) {
    if (p.length < 11) return null;
    let fw = '';
    for (let i = 11; i < p.length; i++) fw += String.fromCharCode(p[i]);
    return { type: 'hello', proto: dv.getUint8(0), sensors: dv.getUint8(1), odr: dv.getFloat32(2, true),
      decim: dv.getUint8(6), scale: dv.getFloat32(7, true), fw };
  }

  // Decode a payload by frame type. Returns an object with `type` as a lowercase name, or null.
  function decodePayload(type, p) {
    const dv = new DataView(p.buffer, p.byteOffset, p.byteLength);
    switch (type) {
      case T.DATA: return decodeData(dv, p);
      case T.RAW: return decodeRaw(dv, p);
      case T.STATUS: return decodeStatus(dv, p);
      case T.EVENT: return decodeEvent(dv, p);
      case T.HEALTH: return decodeHealth(dv, p);
      case T.PARAMS: return decodeParams(dv, p);
      case T.HELLO: return decodeHello(dv, p);
      case T.LOG: {
        let s = '';
        for (let i = 0; i < p.length; i++) s += String.fromCharCode(p[i]);
        return { type: 'log', text: s };
      }
      default: return { type: 'unknown', frameType: type, payload: p };
    }
  }

  // Decode one complete frame body (after COBS decoding). Returns {ok, frame|reason}.
  function decodeBody(body) {
    if (!body || body.length < 5) return { ok: false, reason: 'short' };
    const crc = body[body.length - 2] | (body[body.length - 1] << 8);
    if (crc16(body, 0, body.length - 2) !== crc) return { ok: false, reason: 'crc' };
    const type = body[0], seq = body[1] | (body[2] << 8);
    const f = decodePayload(type, body.subarray(3, body.length - 2));
    if (!f) return { ok: false, reason: 'payload' };
    f.seq = seq; f.ftype = type;
    return { ok: true, frame: f };
  }

  // Streaming parser. Feed arbitrary byte chunks with push(); onFrame(frame) is called per valid frame.
  // Stats: frames, bad (CRC/COBS/payload errors), lost (frames missing by seq gaps; jumps > 1000 are a
  // device restart and are not counted), restarts.
  class Parser {
    constructor(onFrame) {
      this.onFrame = onFrame || function () {};
      this.buf = new Uint8Array(4096);
      this.len = 0;
      this.synced = false; // drop bytes until the first delimiter so a mid-frame start is not an error
      this.maxSeq = null;
      this.pending = new Set(); // seqs skipped over that may still arrive (the firmware can enqueue
                                // two adjacent frames slightly out of order from its two cores)
      this.confirmedLost = 0;
      this.stats = { frames: 0, bad: 0, lost: 0, reordered: 0, restarts: 0, bytes: 0 };
    }
    reset() { this.len = 0; this.synced = false; this.maxSeq = null; this.pending.clear(); }
    _seq(seq) {
      const s = this.stats;
      if (this.maxSeq === null) { this.maxSeq = seq; return; }
      const d = (seq - this.maxSeq) & 0xFFFF;
      if (d === 0) return;
      if (d < 1000) {                       // forward, possibly skipping frames
        for (let k = 1; k < d; k++) this.pending.add((this.maxSeq + k) & 0xFFFF);
        this.maxSeq = seq;
        for (const q of this.pending) {     // anything > 64 behind is lost for good
          if (((this.maxSeq - q) & 0xFFFF) > 64) { this.pending.delete(q); this.confirmedLost++; }
        }
      } else if (d > 0xFFFF - 1000) {       // slightly backwards: a late frame, not a restart
        if (this.pending.delete(seq)) s.reordered++;
      } else {                              // big jump: device restart, not loss
        s.restarts++; this.pending.clear(); this.maxSeq = seq;
      }
      s.lost = this.confirmedLost + this.pending.size;
    }
    push(chunk) {
      this.stats.bytes += chunk.length;
      for (let i = 0; i < chunk.length; i++) {
        const b = chunk[i];
        if (b === 0) {
          if (this.synced && this.len > 0) this._frame(this.buf.subarray(0, this.len));
          this.synced = true;
          this.len = 0;
        } else if (this.synced) {
          if (this.len >= this.buf.length) { this.len = 0; this.synced = false; this.stats.bad++; continue; }
          this.buf[this.len++] = b;
        }
      }
    }
    _frame(block) {
      const r = decodeBody(cobsDecode(block));
      if (!r.ok) { this.stats.bad++; return; }
      const f = r.frame;
      this._seq(f.seq);
      this.stats.frames++;
      this.onFrame(f);
    }
  }

  // Which checks failed, as [{name, bit}] (only checks that were evaluated).
  function failedChecks(pass, ev) {
    const out = [];
    for (let i = 0; i < CHECKS.length; i++) if ((ev & (1 << i)) && !(pass & (1 << i))) out.push(CHECKS[i]);
    return out;
  }

  return { T, TYPE_NAMES, STATES, CHECKS, PARAM_NAMES, P, OUT_SCALE, crc16, cobsEncode, cobsDecode,
    encodeFrame, decodeBody, decodePayload, Cmd, Parser, failedChecks };
});
