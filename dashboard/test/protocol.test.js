// Usage: node test/protocol.test.js [capture.bin]
// The round-trip tests need no capture. With a capture path (raw bytes from the ESP32), the stream
// tests also run: CRC-valid DATA for both sensors, fs near 200 Hz, no index gaps, HEALTH sane.
'use strict';
const fs = require('fs');
const assert = require('assert');
const P = require('../protocol.js');

let passed = 0;
function test(name, fn) {
  try { fn(); passed++; console.log('ok   ' + name); }
  catch (e) { console.log('FAIL ' + name + '\n     ' + e.message); process.exitCode = 1; }
}
const cat = list => new Uint8Array(Buffer.concat(list.map(x => Buffer.from(x))));
function wr(size, fill) { const b = Buffer.alloc(size); fill(b); return new Uint8Array(b); }
function parseAll(bytes) {
  const out = [];
  const p = new P.Parser(f => out.push(f));
  p.push(bytes);
  return { out, stats: p.stats };
}

// ---------------------------------------------------------------- self-contained
test('crc16 CCITT check value', () => {
  assert.strictEqual(P.crc16(Buffer.from('123456789')), 0x29B1);
});

test('COBS round trip incl. zeros and 254+ byte runs', () => {
  const cases = [[], [0], [1, 2, 3], [0, 0, 0], new Array(300).fill(7), new Array(254).fill(1),
    new Array(255).fill(0).map((_, i) => i % 5), [1, 0, 2, 0, 0, 3]];
  for (const c of cases) {
    const enc = P.cobsEncode(Uint8Array.from(c));
    assert.ok(!enc.includes(0), 'encoded data contains a zero');
    assert.deepStrictEqual(Array.from(P.cobsDecode(enc)), c);
  }
});

test('encode/decode STATUS, EVENT, PARAMS, HELLO, LOG', () => {
  const status = wr(45, b => {
    b[0] = 3; b[1] = 4; b[2] = 0x07; b[3] = 0x3F; b[4] = 0x3F;
    [0.12, 0.3, 2.5, 4.7, 0.9, 0.8, 0.95, 0.05, 0.1, 3.2].forEach((v, i) => b.writeFloatLE(v, 5 + 4 * i));
  });
  const event = wr(56, b => {
    b.writeUInt32LE(7, 0); b[4] = 1; b[5] = 0x3F; b[6] = 0x3F; b[7] = 3;
    [8, 0.33, 0.15, 4.72, 1.6, 0.9, 0.8, 0.5, 0.1, 6.1, 2.3].forEach((v, i) => b.writeFloatLE(v, 8 + 4 * i));
    b.writeUInt32LE(12345, 52);
  });
  const params = wr(13, b => { b[0] = 3; [0.04, 0.025, 0.8].forEach((v, i) => b.writeFloatLE(v, 1 + 4 * i)); });
  const hello = wr(16, b => { b[0] = 1; b[1] = 2; b.writeFloatLE(800, 2); b[6] = 4; b.writeFloatLE(2000, 7); b.write('0.1.0', 11); });
  const log = new Uint8Array(Buffer.from('hello world'));
  const { out, stats } = parseAll(cat([[0],
    P.encodeFrame(P.T.STATUS, 10, status), P.encodeFrame(P.T.EVENT, 11, event),
    P.encodeFrame(P.T.PARAMS, 12, params), P.encodeFrame(P.T.HELLO, 13, hello),
    P.encodeFrame(P.T.LOG, 14, log)]));
  assert.strictEqual(out.length, 5);
  assert.strictEqual(stats.bad, 0);
  assert.strictEqual(stats.lost, 0);
  const [s, e, pa, h, l] = out;
  assert.strictEqual(s.type, 'status'); assert.strictEqual(s.state, 3); assert.strictEqual(s.level, 4);
  assert.ok(s.ok1 && s.ok2 && s.silenced); assert.strictEqual(s.passMask, 0x3F);
  assert.ok(Math.abs(s.freq - 4.7) < 1e-5); assert.ok(Math.abs(s.dur - 3.2) < 1e-5);
  assert.strictEqual(e.type, 'event'); assert.strictEqual(e.id, 7); assert.ok(e.accepted);
  assert.strictEqual(e.level, 3); assert.ok(Math.abs(e.freq - 4.72) < 1e-5); assert.strictEqual(e.tStart, 12345);
  assert.ok(Math.abs(e.disp - 2.3) < 1e-5);
  assert.strictEqual(pa.n, 3); assert.ok(Math.abs(pa.values[1] - 0.025) < 1e-6);
  assert.strictEqual(h.fw, '0.1.0'); assert.strictEqual(h.odr, 800); assert.strictEqual(h.decim, 4);
  assert.strictEqual(l.text, 'hello world');
  assert.strictEqual(s.seq, 10); assert.strictEqual(l.seq, 14);
});

test('encode/decode DATA and HEALTH', () => {
  const n = 3, SZ = 39;
  const data = wr(15 + n * 12, b => {
    b[0] = 1; b[1] = 1; b[2] = n; b.writeUInt32LE(1000, 3); b.writeUInt32LE(4000000000, 7); b.writeFloatLE(207.5, 11);
    for (let i = 0; i < n * 6; i++) b.writeInt16LE(i % 2 ? -(i * 100) : i * 100, 15 + 2 * i);
  });
  const health = wr(14 + 2 * SZ, b => {
    b.writeUInt32LE(5000, 0); b.writeUInt32LE(2, 4); b.writeUInt32LE(150000, 8); b.writeUInt16LE(4900, 12);
    for (let s = 0; s < 2; s++) {
      const o = 14 + s * SZ;
      b[o] = 1; b[o + 1] = 0xE5; b[o + 2] = 0xE5; b[o + 3] = 1;
      b.writeFloatLE(s ? 788.2 : 829.1, o + 10); b.writeUInt32LE(99, o + 14); b.writeUInt32LE(3 + s, o + 18);
      b.writeUInt32LE(1, o + 22); b.writeFloatLE(5.5, o + 30); b.writeFloatLE(2.5, o + 34); b[o + 38] = 1;
    }
  });
  const { out, stats } = parseAll(cat([[0], P.encodeFrame(P.T.DATA, 65535, data), P.encodeFrame(P.T.HEALTH, 0, health)]));
  assert.strictEqual(stats.bad, 0);
  assert.strictEqual(out[0].type, 'data');
  assert.strictEqual(out[0].n, 3); assert.ok(out[0].gap); assert.ok(!out[0].zeroed);
  assert.strictEqual(out[0].tUs, 4000000000); assert.strictEqual(out[0].idx, 1000);
  assert.ok(Math.abs(out[0].fs - 207.5) < 1e-4);
  assert.strictEqual(out[0].samples[1], -100); assert.strictEqual(out[0].samples[2], 200);
  assert.strictEqual(out[1].type, 'health');
  assert.ok(Math.abs(out[1].sensors[0].odr - 829.1) < 0.01); assert.strictEqual(out[1].sensors[1].overruns, 4);
  assert.ok(Math.abs(out[1].sensors[1].noiseMg - 5.5) < 1e-5); assert.strictEqual(out[1].heap, 150000);
  assert.strictEqual(out[1].seq, 0);
  assert.strictEqual(stats.lost, 0, 'seq 65535 -> 0 wraps with no loss');
});

test('Parser: chunked input, CRC corruption, mid-frame start, seq gap vs restart', () => {
  const seqs = [1, 2, 3, 4, 5, 9]; // 6..8 missing -> lost 3
  const frames = seqs.map((q, i) => P.encodeFrame(P.T.LOG, q, Uint8Array.from([65 + i, 0, 66])));
  const stream = cat([[5, 6, 7, 0], ...frames]); // leading garbage = start in the middle of a frame
  const out = [];
  const p = new P.Parser(f => out.push(f));
  for (let i = 0; i < stream.length; i += 3) p.push(stream.subarray(i, i + 3));
  assert.strictEqual(out.length, 6);
  assert.strictEqual(p.stats.lost, 3);
  assert.strictEqual(p.stats.bad, 0);
  assert.strictEqual(out[0].text, 'A\0B');
  // corrupt one payload byte
  const bad = Uint8Array.from(frames[0]); bad[4] ^= 0x55;
  const r2 = parseAll(cat([[0], bad, frames[1]]));
  assert.strictEqual(r2.out.length, 1); assert.strictEqual(r2.stats.bad, 1);
  // restart: a huge seq jump is not counted as loss
  const r3 = parseAll(cat([[0], P.encodeFrame(P.T.LOG, 4000, [65]), P.encodeFrame(P.T.LOG, 3, [66])]));
  assert.strictEqual(r3.stats.lost, 0); assert.strictEqual(r3.stats.restarts, 1);
  // two adjacent frames swapped (seen on the real device): reordered, not lost
  const r4 = parseAll(cat([[0], ...[10, 12, 11, 13, 14].map(q => P.encodeFrame(P.T.LOG, q, [65]))]));
  assert.strictEqual(r4.out.length, 5); assert.strictEqual(r4.stats.lost, 0); assert.strictEqual(r4.stats.reordered, 1);
});

test('command encoders decode as valid frames with the right payloads', () => {
  const dec = bytes => {
    assert.strictEqual(bytes[bytes.length - 1], 0);
    const r = P.decodeBody(P.cobsDecode(bytes.subarray(0, bytes.length - 1)));
    assert.ok(r.ok, r.reason || 'decode');
    return r.frame;
  };
  const sp = dec(P.Cmd.setParam(2, 0.8));
  assert.strictEqual(sp.ftype, 0x80); assert.strictEqual(sp.payload[0], 2);
  assert.ok(Math.abs(new DataView(sp.payload.buffer, sp.payload.byteOffset).getFloat32(1, true) - 0.8) < 1e-6);
  assert.strictEqual(dec(P.Cmd.rezero()).payload[0], 3);
  assert.strictEqual(dec(P.Cmd.rawMode(true)).payload[0], 1);
  assert.strictEqual(dec(P.Cmd.testAlarm()).ftype, 0x83);
  assert.strictEqual(dec(P.Cmd.silence()).ftype, 0x84);
  assert.strictEqual(dec(P.Cmd.getParams()).ftype, 0x86);
  assert.strictEqual(dec(P.Cmd.resetParams()).ftype, 0x87);
  assert.strictEqual(dec(P.Cmd.setCal(1, [0, 0, 0], [1, 1, 1])).payload.length, 25);
  // same bytes as tools/monitor.py make_cmd(0x86): COBS(type, seq 0, crc) + 0x00
  const c = P.crc16(Uint8Array.from([0x86, 0, 0]));
  assert.deepStrictEqual(Array.from(P.Cmd.getParams()),
    Array.from(P.cobsEncode(Uint8Array.from([0x86, 0, 0, c & 255, c >> 8]))).concat([0]));
});

// ---------------------------------------------------------------- captured stream
const cap = process.argv[2];
if (!cap) {
  console.log('(no capture path given: skipping stream tests)');
} else {
  const bytes = new Uint8Array(fs.readFileSync(cap));
  const { out: frames, stats } = parseAll(bytes);
  const of = t => frames.filter(f => f.type === t);
  console.log(`capture: ${bytes.length} bytes, ${frames.length} frames, bad ${stats.bad}, lost ${stats.lost}`);

  test('capture: CRC-valid frames, nothing corrupt', () => {
    assert.ok(frames.length > 100, 'too few frames: ' + frames.length);
    assert.strictEqual(stats.bad, 0, 'bad frames: ' + stats.bad);
  });
  test('capture: no lost frames (seq)', () => assert.strictEqual(stats.lost, 0));
  test('capture: DATA for both sensors, fs within 0.8x-1.2x of 200 Hz', () => {
    for (const s of [0, 1]) {
      const d = of('data').filter(f => f.sensor === s);
      assert.ok(d.length > 50, `sensor ${s + 1}: only ${d.length} DATA frames`);
      for (const f of d) assert.ok(f.fs > 160 && f.fs < 240, `fs ${f.fs}`);
      const mean = d.reduce((a, f) => a + f.fs, 0) / d.length;
      console.log(`     S${s + 1}: ${d.length} frames, mean fs ${mean.toFixed(2)} Hz`);
    }
  });
  test('capture: no index gaps per sensor, time base consistent with fs', () => {
    for (const s of [0, 1]) {
      const d = of('data').filter(f => f.sensor === s);
      for (let i = 1; i < d.length; i++) {
        assert.strictEqual(d[i].idx, d[i - 1].idx + d[i - 1].n, `S${s + 1} index gap at frame ${i}`);
        assert.ok(!d[i].gap, 'gap flag set');
      }
      const last = d[d.length - 1];
      const nBefore = last.idx - d[0].idx;
      const dt = ((last.tUs - d[0].tUs) >>> 0) / 1e6;
      const rate = nBefore / dt;
      assert.ok(Math.abs(rate / last.fs - 1) < 0.02, `time base rate ${rate.toFixed(2)} vs fs ${last.fs.toFixed(2)}`);
    }
  });
  test('capture: DATA samples plausible (az ~ +1 g)', () => {
    for (const s of [0, 1]) {
      const d = of('data').filter(f => f.sensor === s).pop();
      let az = 0;
      for (let i = 0; i < d.n; i++) az += d.samples[6 * i + 2] / P.OUT_SCALE;
      az /= d.n;
      assert.ok(az > 0.8 && az < 1.2, `S${s + 1} mean az ${az.toFixed(3)}`);
    }
  });
  test('capture: HEALTH parsed, odr ~829 and ~788 Hz', () => {
    const h = of('health').pop();
    assert.ok(h, 'no HEALTH frame');
    assert.ok(of('health').length >= 5, 'expected ~1 Hz HEALTH');
    const [a, b] = h.sensors;
    console.log(`     odr ${a.odr.toFixed(1)} / ${b.odr.toFixed(1)} Hz, noise ${a.noiseMg.toFixed(2)} / ${b.noiseMg.toFixed(2)} mg,` +
      ` overruns ${a.overruns}/${b.overruns}, i2c ${a.i2c}/${b.i2c}, present ${a.present}/${b.present}`);
    assert.ok(a.present && b.present);
    assert.ok(a.odr > 800 && a.odr < 860, 'S1 odr ' + a.odr);
    assert.ok(b.odr > 760 && b.odr < 820, 'S2 odr ' + b.odr);
  });
  test('capture: STATUS ~10 Hz with plausible fields; HELLO/PARAMS decode if present', () => {
    const st = of('status');
    assert.ok(st.length >= 80 && st.length <= 110, 'status count ' + st.length);
    for (const s of st) assert.ok(s.state >= 0 && s.state <= 4 && s.rms >= 0 && s.rms < 20);
    for (const h of of('hello')) assert.strictEqual(h.scale, 2000);
    for (const p of of('params')) assert.strictEqual(p.n, 18);
  });
}
console.log(`${passed} tests passed` + (process.exitCode ? ', some FAILED' : ''));
